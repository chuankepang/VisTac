#include <iostream>
#include <cmath>
#include <thread>
#include <mutex>
#include <array>
#include <deque>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <memory>
#include <algorithm> // 引入 std::max

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
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        cart_positions_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "target_cartpos", qos, 
            std::bind(&rt_RobotCtrlNode::cartPositionCallback, this, std::placeholders::_1)
        );

        motion_controller_ = robot_.getRtMotionController().lock();
        robot_.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m, RtSupportedFields::tcpPose_m});

        std::array<double, 7> cur_joints {};
        robot_.getStateData(RtSupportedFields::jointPos_m, cur_joints);
        motion_controller_->MoveJ(0.5, cur_joints, zero_pos);
        RCLCPP_INFO(this->get_logger(), "Robot at ready position.");

        keyboard_thread_ = std::thread([this]() { this->keyboard_input_thread(); });
    }

    ~rt_RobotCtrlNode()
    {
        if (control_thread_.joinable()) {
            motion_controller_->stopLoop();
            control_thread_.join();
        }
        std::error_code ec;
        robot_.setPowerState(false, ec);
    }

private:
    const double dt = 0.001;              
    const double max_vel_p = 1.0;

    std::mutex target_mutex_;
    Eigen::Vector3d target_p_, curr_p_;
    Eigen::Quaterniond target_q_, curr_q_;

    // --- S 曲线规划器相关状态变量 ---
    std::shared_ptr<rokae::CartMotionGenerator> cart_s_ = nullptr;
    Eigen::Vector3d start_p_, end_p_;
    Eigen::Quaterniond start_q_, end_q_;
    
    // 新增：记录当前规划器正在前往的“活跃目标”，用于过滤高频重复指令
    Eigen::Vector3d active_target_p_;
    Eigen::Quaterniond active_target_q_;
    
    double motion_time_ = 0.0;
    double total_s_ = 0.0;

    void cartPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() == 16) {
            Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> f_mat(msg->data.data());
            Eigen::Matrix4d d_mat = f_mat.cast<double>();
            
            std::lock_guard<std::mutex> lock(target_mutex_);
            target_p_ = d_mat.block<3, 1>(0, 3);
            target_q_ = Eigen::Quaterniond(d_mat.block<3, 3>(0, 0));
            target_q_.normalize();
            // 注意：这里不再无脑设定 update 标志位，甄别工作交给底层的 1kHz Loop
        }
    }

    CartesianPosition rokae_callback()
    {
        CartesianPosition cmd;
        Eigen::Vector3d req_p;
        Eigen::Quaterniond req_q;

        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            req_p = target_p_;
            req_q = target_q_;
        }

        // 1. 甄别目标是否发生实质性变化 (位置差 > 0.1mm 或 姿态差 > 0.05度)
        double pos_diff = (req_p - active_target_p_).norm();
        double ori_diff = active_target_q_.angularDistance(req_q);

        if (pos_diff > 1e-4 || ori_diff > 1e-3) {
            // 目标确实更新了，打断当前动作，以当前位置为起点重新规划 S 曲线
            start_p_ = curr_p_;
            start_q_ = curr_q_;
            end_p_ = req_p;
            end_q_ = req_q;
            
            // 更新活跃目标
            active_target_p_ = req_p;
            active_target_q_ = req_q;
            
            total_s_ = (end_p_ - start_p_).norm();
            
            // 防止纯旋转动作导致 total_s_ 为 0，使得 SDK 规划器除零报错，给一个 0.01mm 的保底值
            double plan_s = std::max(total_s_, 1e-5);
            
            cart_s_ = std::make_shared<rokae::CartMotionGenerator>(max_vel_p, plan_s);
            cart_s_->calculateSynchronizedValues(0);
            motion_time_ = 0.0;
        }

        // 2. 利用官方规划器插值当前周期的位姿
        if (cart_s_ != nullptr) {
            motion_time_ += dt; 
            double delta_s = 0.0;
            
            // !calculateDesiredValues 正常运行返回 true (注: 你的原厂例程中取了反向逻辑)
            if (!cart_s_->calculateDesiredValues(motion_time_, &delta_s)) {
                if (total_s_ > 1e-6) {
                    curr_p_ = start_p_ + (end_p_ - start_p_) * (delta_s / total_s_);
                    curr_q_ = start_q_.slerp(delta_s / total_s_, end_q_);
                } else {
                    // 应对纯旋转情况，借用 delta_s 比例进行姿态插值
                    curr_q_ = start_q_.slerp(delta_s / 1e-5, end_q_);
                }
                curr_q_.normalize();
            } else {
                // 运动到位，锁定在终点并销毁规划器
                curr_p_ = end_p_;
                curr_q_ = end_q_;
                cart_s_ = nullptr;
            }
        }

        // 3. 构造底层输出指令
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
                        curr_p_ = mat.block<3, 1>(0, 3);
                        curr_q_ = Eigen::Quaterniond(mat.block<3, 3>(0, 0));
                        curr_q_.normalize();
                        target_p_ = curr_p_;
                        target_q_ = curr_q_;
                        
                        // 初始化时，当前点即为活跃目标点
                        active_target_p_ = curr_p_;
                        active_target_q_ = curr_q_;
                        cart_s_ = nullptr;
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
                    RCLCPP_INFO(this->get_logger(), "Smooth S-Line control loop started.");
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
        fcntl(STDIN_FILENO, F_SETFL, oldf);
        if (ch != EOF) { ungetc(ch, stdin); return 1; }
        return 0;
    }

    rokae::xMateErProRobot robot_;
    std::shared_ptr<RtMotionControlCobot<7>> motion_controller_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cart_positions_sub_;
    const std::array<double, 7> zero_pos = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};
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