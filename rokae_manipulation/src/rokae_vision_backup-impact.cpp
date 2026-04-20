#include <iostream>
#include <cmath>
#include <thread>
#include <mutex>
#include <array>
#include <deque>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

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
    const double max_vel_p = 0.8;         
    const double max_acc_p = 2.0;         
    const double max_vel_r = 1.5;         
    const double max_acc_r = 4.0;         
    const double filter_gain = 20.0;      

    std::mutex target_mutex_;
    Eigen::Vector3d target_p_, curr_p_, curr_v_p_;
    Eigen::Quaterniond target_q_, curr_q_;
    Eigen::Vector3d curr_omega_;          

    void cartPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() == 16) {
            // 修改点 1: 先映射为 float 矩阵，再 cast 为 double，解决 float* 无法赋值给 double* 的问题
            Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> f_mat(msg->data.data());
            Eigen::Matrix4d d_mat = f_mat.cast<double>();
            
            std::lock_guard<std::mutex> lock(target_mutex_);
            target_p_ = d_mat.block<3, 1>(0, 3);
            target_q_ = Eigen::Quaterniond(d_mat.block<3, 3>(0, 0));
            target_q_.normalize();
        }
    }

    CartesianPosition rokae_callback()
    {
        CartesianPosition cmd;
        Eigen::Vector3d l_target_p;
        Eigen::Quaterniond l_target_q;

        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            l_target_p = target_p_;
            l_target_q = target_q_;
        }

        // 1. 位置插值
        Eigen::Vector3d err_p = l_target_p - curr_p_;
        Eigen::Vector3d v_des = err_p * filter_gain; 
        if (v_des.norm() > max_vel_p) v_des = v_des.normalized() * max_vel_p;
        
        Eigen::Vector3d a_req = (v_des - curr_v_p_) / dt;
        if (a_req.norm() > max_acc_p) a_req = a_req.normalized() * max_acc_p;

        curr_v_p_ += a_req * dt;
        curr_p_ += curr_v_p_ * dt;

        // 2. 姿态插值
        if (curr_q_.dot(l_target_q) < 0.0) l_target_q.coeffs() = -l_target_q.coeffs();
        Eigen::Quaterniond q_diff = l_target_q * curr_q_.inverse();
        Eigen::AngleAxisd aa_err(q_diff);
        
        double angle = aa_err.angle();
        if (angle > M_PI) angle -= 2.0 * M_PI;
        
        // 修改点 2: 使用 if-else 代替三元运算符，解决 Eigen 表达式类型推导失败
        Eigen::Vector3d omega_des;
        if (std::abs(angle) < 1e-6) {
            omega_des.setZero();
        } else {
            omega_des = aa_err.axis() * angle * filter_gain;
        }

        if (omega_des.norm() > max_vel_r) omega_des = omega_des.normalized() * max_vel_r;

        Eigen::Vector3d alpha_req = (omega_des - curr_omega_) / dt;
        if (alpha_req.norm() > max_acc_r) alpha_req = alpha_req.normalized() * max_acc_r;

        curr_omega_ += alpha_req * dt;
        if (curr_omega_.norm() > 1e-6) {
            Eigen::AngleAxisd delta_rot(curr_omega_.norm() * dt, curr_omega_.normalized());
            curr_q_ = Eigen::Quaterniond(delta_rot) * curr_q_;
            curr_q_.normalize();
        }

        // 3. 输出
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> out_mat = Eigen::Matrix4d::Identity();
        out_mat.block<3, 3>(0, 0) = curr_q_.toRotationMatrix();
        out_mat.block<3, 1>(0, 3) = curr_p_;

        // 修改点 3: 将 out_mat 数据拷贝到 cmd.pos
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
                        curr_v_p_.setZero();
                        curr_omega_.setZero();
                    }

                    // 修改点 4: 显式指定 std::function 类型，解决模板推导失败
                    motion_controller_->setControlLoop(
                        std::function<CartesianPosition()>(std::bind(&rt_RobotCtrlNode::rokae_callback, this)), 
                        0, true);

                    motion_controller_->startMove(RtControllerMode::cartesianPosition);
                    
                    control_thread_ = std::thread([this]() {
                        try { this->motion_controller_->startLoop(true); }
                        catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "Loop error: %s", e.what()); }
                    });
                    control_loop_started_ = true;
                    RCLCPP_INFO(this->get_logger(), "Control loop started.");
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