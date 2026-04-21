#include <iostream>
#include <cmath>
#include <thread>
#include <mutex>
#include <array>
#include <deque>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>

// 引入 Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rokae_rt/robot.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;
using namespace rokae;

class rt_RobotCtrlNode : public rclcpp::Node
{
public:
    rt_RobotCtrlNode() : Node("rt_robot_control_node")
    {
        // ====================================================================
        // 参数联动计算核心：修改下方 target_ros_hz_ 即可自动联动所有时间参数
        // ====================================================================
        target_ros_hz_ = 5.0; // 模型下发频率 (Hz)
        
        // 根据频率自动计算插值周期 (ms)
        double period_ms = 1000.0 / target_ros_hz_; 
        // 引入 10% 的时间缓冲 (Jitter Buffer)，对抗网络波动和推理延迟
        // 例如：5Hz 理论周期 200ms，缓冲后为 220ms。如果在 220ms 内新指令没来，
        // 机械臂会极其平滑地减速停在终点，绝不会乱跑。
        interp_duration_ = std::chrono::milliseconds(static_cast<int>(period_ms * 1.10));
        
        RCLCPP_INFO(this->get_logger(), "Target ROS Freq: %.1f Hz, Interp Duration: %ld ms", 
                    target_ros_hz_, interp_duration_.count());
        // ====================================================================

        std::error_code ec;
        try {
            std::string robot_ip = "192.168.0.160";
            std::string local_ip = "192.168.0.100";
            robot_.connectToRobot(robot_ip, local_ip);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Connection failed: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        robot_.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot_.setRtNetworkTolerance(20, ec);
        robot_.setOperateMode(OperateMode::automatic, ec);
        robot_.setPowerState(true, ec);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)); 
        //qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); 
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT); 
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        cart_positions_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "target_cartpos", qos, 
            std::bind(&rt_RobotCtrlNode::cartPositionCallback, this, std::placeholders::_1)
        );

        motion_controller_ = robot_.getRtMotionController().lock();
        robot_.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m, RtSupportedFields::tcpPose_m});

        std::array<double, 7> cur_joints {};
        robot_.getStateData(RtSupportedFields::jointPos_m, cur_joints);
        motion_controller_->MoveJ(0.5, cur_joints, ready_pos_);
        RCLCPP_INFO(this->get_logger(), "Robot at ready position. Press 'c' to start RT loop.");

        keyboard_thread_ = std::thread([this]() { this->keyboard_input_thread(); });
    }

    ~rt_RobotCtrlNode()
    {
        if (control_thread_.joinable()) {
            motion_controller_->stopLoop();
            control_thread_.join();
        }
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        std::error_code ec;
        robot_.setPowerState(false, ec);
    }

private:
    // --- 联动参数区 ---
    double target_ros_hz_; 
    std::chrono::milliseconds interp_duration_;

    // --- 底层控制参数 (1kHz 滤波器参数) ---
    const double dt = 0.001;              
    const double omega_n = 40.0;          // 二阶阻尼自然频率，兼顾跟手与抗突变

    // 物理限制
    const double max_vel_p = 0.5;         // m/s
    const double max_acc_p = 2.0;         // m/s^2
    const double max_vel_r = 2.0;         // rad/s
    const double max_acc_r = 10.0;        // rad/s^2

    // --- 状态变量区 ---
    std::mutex target_mutex_;
    
    // 宏观插值轨迹 (由 ROS 触发更新)
    Eigen::Vector3d start_p_, end_p_;
    Eigen::Quaterniond start_q_, end_q_;
    std::chrono::time_point<std::chrono::steady_clock> start_time_, end_time_;

    // 微观机械臂状态 (在 1kHz 循环中实时流转)
    Eigen::Vector3d curr_p_, curr_v_p_;
    Eigen::Quaterniond curr_q_;
    Eigen::Vector3d curr_omega_;          

    // --- 回调函数 1：模型指令接收 (低频 5Hz) ---
    void cartPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() == 16) {
            Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> f_mat(msg->data.data());
            Eigen::Matrix4d d_mat = f_mat.cast<double>();
            
            Eigen::Vector3d new_p = d_mat.block<3, 1>(0, 3);
            Eigen::Quaterniond new_q = Eigen::Quaterniond(d_mat.block<3, 3>(0, 0));
            new_q.normalize();

            std::lock_guard<std::mutex> lock(target_mutex_);
            
            // 核心灵魂：强行从当前的“微观滤波位置”作为新周期的起点
            // 无论上一个动作有没有完成，无论网络丢了多少帧，全盘接纳当前状态！
            start_p_ = curr_p_;
            start_q_ = curr_q_;
            
            end_p_ = new_p;
            end_q_ = new_q;
            
            // 四元数防绕远路处理
            if (start_q_.dot(end_q_) < 0.0) {
                end_q_.coeffs() *= -1.0;
            }
            
            // 刷新时间戳
            start_time_ = std::chrono::steady_clock::now();
            end_time_ = start_time_ + interp_duration_;
        }
    }

    // --- 回调函数 2：Rokae 底层控制 (极高频 1kHz) ---
    CartesianPosition rokae_callback()
    {
        CartesianPosition cmd;
        Eigen::Vector3d l_interp_p;
        Eigen::Quaterniond l_interp_q;

        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            
            // 1. 宏观轨迹插值：计算当前时刻在橡皮筋上的进度 alpha
            auto now = std::chrono::steady_clock::now();
            double alpha = 1.0; // 默认静止在终点
            
            if (now < end_time_ && start_time_ < end_time_) {
                std::chrono::duration<double> elapsed = now - start_time_;
                std::chrono::duration<double> total = end_time_ - start_time_;
                alpha = elapsed.count() / total.count();
                alpha = std::max(0.0, std::min(1.0, alpha)); // 严格钳制在 [0, 1]
            }

            // Lerp 线性插值 & Slerp 球面插值
            l_interp_p = start_p_ + alpha * (end_p_ - start_p_);
            l_interp_q = start_q_.slerp(alpha, end_q_);
        }

        // 2. 微观滤波：二阶临界阻尼追踪这个 l_interp_p (原本你的精髓代码)
        
        // -- 位置控制 --
        Eigen::Vector3d err_p = l_interp_p - curr_p_;
        Eigen::Vector3d a_des = (omega_n * omega_n) * err_p - (2.0 * omega_n) * curr_v_p_;

        if (a_des.norm() > max_acc_p) a_des = a_des.normalized() * max_acc_p;
        
        curr_v_p_ += a_des * dt;
        if (curr_v_p_.norm() > max_vel_p) curr_v_p_ = curr_v_p_.normalized() * max_vel_p;
        curr_p_ += curr_v_p_ * dt;

        // -- 姿态控制 --
        if (curr_q_.dot(l_interp_q) < 0.0) l_interp_q.coeffs() = -l_interp_q.coeffs();
        
        Eigen::Quaterniond q_diff = l_interp_q * curr_q_.inverse();
        Eigen::AngleAxisd aa_err(q_diff);
        double angle = aa_err.angle();
        if (angle > M_PI) angle -= 2.0 * M_PI;

        Eigen::Vector3d omega_des = Eigen::Vector3d::Zero();
        if (std::abs(angle) > 1e-6) {
            omega_des = aa_err.axis() * angle * omega_n; 
        }

        Eigen::Vector3d alpha_req = (omega_n) * (omega_des - curr_omega_);
        if (alpha_req.norm() > max_acc_r) alpha_req = alpha_req.normalized() * max_acc_r;

        curr_omega_ += alpha_req * dt;
        if (curr_omega_.norm() > max_vel_r) curr_omega_ = curr_omega_.normalized() * max_vel_r;

        if (curr_omega_.norm() > 1e-6) {
            Eigen::AngleAxisd delta_rot(curr_omega_.norm() * dt, curr_omega_.normalized());
            curr_q_ = Eigen::Quaterniond(delta_rot) * curr_q_;
            curr_q_.normalize();
        }

        // 3. 构建输出给机器人
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> out_mat = Eigen::Matrix4d::Identity();
        out_mat.block<3, 3>(0, 0) = curr_q_.toRotationMatrix();
        out_mat.block<3, 1>(0, 3) = curr_p_;

        std::copy(out_mat.data(), out_mat.data() + 16, cmd.pos.begin());
        return cmd;
    }

    void keyboard_input_thread()
    {
        while (rclcpp::ok()) {
            if (kbhit()) {
                char ch = getchar();
                if (ch == 'c' && !control_loop_started_) {
                    std::array<double, 16> init_cart {};
                    robot_.getStateData(RtSupportedFields::tcpPose_m, init_cart);
                    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> mat(init_cart.data());
                    
                    {
                        std::lock_guard<std::mutex> lock(target_mutex_);
                        // 初始化微观状态
                        curr_p_ = mat.block<3, 1>(0, 3);
                        curr_q_ = Eigen::Quaterniond(mat.block<3, 3>(0, 0));
                        curr_q_.normalize();
                        curr_v_p_.setZero();
                        curr_omega_.setZero();
                        
                        // 初始化宏观状态：起点终点全对齐当前真实位姿
                        start_p_ = curr_p_;
                        end_p_ = curr_p_;
                        start_q_ = curr_q_;
                        end_q_ = curr_q_;
                        
                        // 将时间指针拨到现在，确保 alpha 初始就是 1.0 (原地待命)
                        start_time_ = std::chrono::steady_clock::now();
                        end_time_ = start_time_; 
                    }

                    motion_controller_->setControlLoop(
                        std::function<CartesianPosition()>(std::bind(&rt_RobotCtrlNode::rokae_callback, this)), 
                        0, true);

                    motion_controller_->startMove(RtControllerMode::cartesianPosition);
                    
                    control_thread_ = std::thread([this]() {
                        try { this->motion_controller_->startLoop(true); }
                        catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "Loop error: %s", e.what()); }
                    });
                    control_loop_started_ = true;
                    RCLCPP_INFO(this->get_logger(), "RT Control loop started smoothly. Ready to receive commands.");
                } else if (ch == 'q') {
                    rclcpp::shutdown();
                    return;
                }
            }
            std::this_thread::sleep_for(10ms);
        }
    }

    int kbhit(void) {
        struct termios oldt, newt;
        int ch, oldf;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, fcntl(STDIN_FILENO, F_GETFL, 0) & ~O_NONBLOCK);
        if (ch != EOF) { ungetc(ch, stdin); return 1; }
        return 0;
    }

    rokae::xMateErProRobot robot_;
    std::shared_ptr<RtMotionControlCobot<7>> motion_controller_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cart_positions_sub_;
    const std::array<double, 7> ready_pos_ = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};
    bool control_loop_started_ = false;
    std::thread control_thread_, keyboard_thread_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rt_RobotCtrlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}