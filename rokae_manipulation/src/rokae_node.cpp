#include <iostream>
#include <thread>
#include <mutex>
#include <array>
#include <termios.h>
#include <fcntl.h>
#include <deque>
#include <stdexcept>
#include <unistd.h>
#include <cmath>
#include "rokae_rt/robot.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "cust_msgs/msg/stampfloat32array.hpp"

using namespace std::chrono_literals;
using namespace rokae;

class rt_RobotCtrlNode : public rclcpp::Node {
public:
    rt_RobotCtrlNode() : Node("rt_robot_control_node") {
        std::error_code ec;
        try {
            std::string robot_ip = "192.168.0.160";
            std::string local_ip = "192.168.0.100";
            robot_.connectToRobot(robot_ip, local_ip, ec);
            if (ec) {
                RCLCPP_ERROR(get_logger(), "Failed to connect to robot at %s: %s", robot_ip.c_str(), ec.message().c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Connection failed");
            }
            RCLCPP_INFO(get_logger(), "Connected to robot at %s", robot_ip.c_str());

            robot_.setMotionControlMode(MotionControlMode::RtCommand, ec);
            if (ec) {
                RCLCPP_ERROR(get_logger(), "Failed to set motion control mode: %s", ec.message().c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Motion control mode failed");
            }
            robot_.setRtNetworkTolerance(20, ec);
            if (ec) {
                RCLCPP_ERROR(get_logger(), "Failed to set network tolerance: %s", ec.message().c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Network tolerance failed");
            }
            robot_.setOperateMode(OperateMode::automatic, ec);
            if (ec) {
                RCLCPP_ERROR(get_logger(), "Failed to set operate mode: %s", ec.message().c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Operate mode failed");
            }
            robot_.setPowerState(true, ec);
            if (ec) {
                RCLCPP_ERROR(get_logger(), "Failed to set power state: %s", ec.message().c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Power state failed");
            }

            PowerState state = robot_.powerState(ec);
            std::string state_str;
            switch (state) {
                case PowerState::on: state_str = "上电"; break;
                case PowerState::off: state_str = "下电"; break;
                case PowerState::unknown: state_str = "未知"; break;
                case PowerState::estop: state_str = "急停"; break;
                case PowerState::gstop: state_str = "安全门打开"; break;
                default: state_str = "无效状态"; break;
            }
            RCLCPP_INFO(get_logger(), "Robot power state: %s", state_str.c_str());
            if (state != PowerState::on) {
                RCLCPP_ERROR(get_logger(), "Robot not powered on, cannot proceed");
                rclcpp::shutdown();
                throw std::runtime_error("Invalid robot state");
            }

            motion_controller_ = robot_.getRtMotionController().lock();
            robot_.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m, RtSupportedFields::cartesianPos});

            std::array<double, 7> cur_pos {};
            robot_.getStateData(RtSupportedFields::jointPos_m, cur_pos, ec);
            if (ec) {
                RCLCPP_ERROR(get_logger(), "Failed to get initial joint positions: %s", ec.message().c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Initial joint position failed");
            }
            last_valid_command_ = cur_pos;
            RCLCPP_INFO(get_logger(), "Current joint positions: %f, %f, %f, %f, %f, %f, %f",
                        cur_pos[0], cur_pos[1], cur_pos[2], cur_pos[3], cur_pos[4], cur_pos[5], cur_pos[6]);

            std::array<double, 6> cur_pose {};
            robot_.getStateData(RtSupportedFields::cartesianPos, cur_pose, ec);
            if (ec) {
                RCLCPP_ERROR(get_logger(), "Failed to get initial cartesian pose: %s", ec.message().c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Initial cartesian pose failed");
            }
            last_valid_pose_ = cur_pose;
            RCLCPP_INFO(get_logger(), "Current cartesian pose: [x: %f, y: %f, z: %f, rx: %f, ry: %f, rz: %f]",
                        cur_pose[0], cur_pose[1], cur_pose[2], cur_pose[3], cur_pose[4], cur_pose[5]);

            std::array<double, 7> target_pos = cur_pos;
            motion_controller_->MoveJ(0.5, cur_pos, target_pos, ec);
            if (ec) {
                RCLCPP_ERROR(get_logger(), "MoveJ failed: %s", ec.message().c_str());
                rclcpp::shutdown();
                throw std::runtime_error("MoveJ failed");
            }
            RCLCPP_INFO(get_logger(), "Robot joint positions initialized.");
            init_move_completed = true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Initialization failed: %s", e.what());
            rclcpp::shutdown();
            throw;
        }

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
            .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
            .deadline(rclcpp::Duration(1ms));

        joint_positions_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "sent_joints", qos,
            std::bind(&rt_RobotCtrlNode::jointPositionCallback, this, std::placeholders::_1));

        force_torque_sub_ = create_subscription<cust_msgs::msg::Stampfloat32array>(
            "force_data", qos,
            std::bind(&rt_RobotCtrlNode::forceTorqueCallback, this, std::placeholders::_1));

        keyboard_thread_ = std::thread([this]() {
            this->keyboard_input_thread();
        });
    }

    ~rt_RobotCtrlNode() {
        stopMotion();
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        std::error_code ec;
        robot_.setPowerState(false, ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to power off robot: %s", ec.message().c_str());
        }
        RCLCPP_INFO(get_logger(), "Robot powered off.");
    }

private:
    struct CartesianPose {
        std::array<double, 6> pose; // [x, y, z, rx, ry, rz]
    };

    void jointPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 7) {
            std::array<double, 7> new_positions;
            for (size_t i = 0; i < 7; i++) {
                new_positions[i] = static_cast<double>(msg->data[i]);
                if (new_positions[i] < -M_PI || new_positions[i] > M_PI) {
                    RCLCPP_WARN(get_logger(), "Joint angle out of range: %f, clamping to [-pi, pi]", new_positions[i]);
                    new_positions[i] = std::clamp(new_positions[i], -M_PI, M_PI);
                }
            }
            {
                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                if (joint_queue_.size() >= max_queue_size_) {
                    joint_queue_.pop_front();
                    RCLCPP_WARN(get_logger(), "Joint queue full, dropping oldest point");
                }
                joint_queue_.push_back(new_positions);
                RCLCPP_INFO(get_logger(), "Joint queue size: %zu", joint_queue_.size());
            }
        } else {
            RCLCPP_WARN(get_logger(), "Received invalid joint data size: %zu", msg->data.size());
        }
    }

    void forceTorqueCallback(const cust_msgs::msg::Stampfloat32array::SharedPtr msg) {
        if (msg->data.size() == 6) {
            std::array<double, 6> new_force_torque;
            for (size_t i = 0; i < 6; i++) {
                new_force_torque[i] = static_cast<double>(msg->data[i]);
                if (std::abs(new_force_torque[i]) > 1000.0) { // 最大力/力矩 1000N/Nm
                    RCLCPP_WARN(get_logger(), "Force/torque %zu out of range: %f, clamping to [-1000, 1000]", i, new_force_torque[i]);
                    new_force_torque[i] = std::clamp(new_force_torque[i], -1000.0, 1000.0);
                }
            }
            {
                std::lock_guard<std::mutex> lock(force_torque_mutex_);
                if (force_queue_.size() >= max_queue_size_) {
                    force_queue_.pop_front();
                    RCLCPP_WARN(get_logger(), "Force queue full, dropping oldest point");
                }
                force_queue_.push_back(new_force_torque);
                last_force_torque_ = new_force_torque;
                RCLCPP_INFO(get_logger(), "Force queue size: %zu, data: [Fx: %f, Fy: %f, Fz: %f, Tx: %f, Ty: %f, Tz: %f]",
                            force_queue_.size(),
                            new_force_torque[0], new_force_torque[1], new_force_torque[2],
                            new_force_torque[3], new_force_torque[4], new_force_torque[5]);
            }
        } else {
            RCLCPP_WARN(get_logger(), "Received invalid force data size: %zu", msg->data.size());
        }
    }

    JointPosition rokae_callback() {
        bool is_ready_to_move = false;
        bool has_new_command = false;
        std::array<double, 7> current_target_joint_pos_ {};
        std::error_code ec;
        {
            std::lock_guard<std::mutex> lock(joint_positions_mutex_);
            RCLCPP_INFO(get_logger(), "Joint queue size: %zu", joint_queue_.size());
            if (!joint_queue_.empty()) {
                current_target_joint_pos_ = joint_queue_.front();
                joint_queue_.pop_front();
                last_valid_command_ = current_target_joint_pos_;
                has_new_command = true;
                RCLCPP_INFO(get_logger(), "Processing joint position: [%f, %f, %f, %f, %f, %f, %f]",
                            current_target_joint_pos_[0], current_target_joint_pos_[1], current_target_joint_pos_[2],
                            current_target_joint_pos_[3], current_target_joint_pos_[4], current_target_joint_pos_[5],
                            current_target_joint_pos_[6]);
            }
            if (!init_joint_pos_set_ && init_move_completed) {
                init_joint_pos_set_ = true;
            }
            is_ready_to_move = init_joint_pos_set_;
        }

        JointPosition cmd;
        if (is_ready_to_move && has_new_command) {
            cmd.joints = std::vector<double>(current_target_joint_pos_.begin(), current_target_joint_pos_.end());
        } else if (is_ready_to_move && !has_new_command) {
            cmd.joints = std::vector<double>(last_valid_command_.begin(), last_valid_command_.end());
        } else {
            std::array<double, 7> current_pos {};
            robot_.getStateData(RtSupportedFields::jointPos_m, current_pos, ec);
            if (ec) {
                RCLCPP_ERROR(get_logger(), "Failed to get current positions: %s", ec.message().c_str());
                rclcpp::shutdown();
            }
            cmd.joints = std::vector<double>(current_pos.begin(), current_pos.end());
        }
        for (double& angle : cmd.joints) {
            if (angle < -M_PI || angle > M_PI) {
                RCLCPP_WARN(get_logger(), "Joint angle out of range: %f, clamping to [-pi, pi]", angle);
                angle = std::clamp(angle, -M_PI, M_PI);
            }
        }
        RCLCPP_INFO(get_logger(), "Sending joint command: [%f, %f, %f, %f, %f, %f, %f]",
                    cmd.joints[0], cmd.joints[1], cmd.joints[2], cmd.joints[3],
                    cmd.joints[4], cmd.joints[5], cmd.joints[6]);
        return cmd;
    }

    CartesianPose admittance_callback() {
        std::array<double, 6> current_force_torque;
        bool has_new_force = false;
        {
            std::lock_guard<std::mutex> lock(force_torque_mutex_);
            if (!force_queue_.empty()) {
                current_force_torque = force_queue_.front();
                force_queue_.pop_front();
                has_new_force = true;
                RCLCPP_INFO(get_logger(), "Processing force/torque: [Fx: %f, Fy: %f, Fz: %f, Tx: %f, Ty: %f, Tz: %f]",
                            current_force_torque[0], current_force_torque[1], current_force_torque[2],
                            current_force_torque[3], current_force_torque[4], current_force_torque[5]);
            } else {
                current_force_torque = last_force_torque_;
            }
        }

        std::array<double, 6> current_pose {};
        std::error_code ec;
        robot_.getStateData(RtSupportedFields::cartesianPos, current_pose, ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to get current cartesian pose: %s", ec.message().c_str());
            return {last_valid_pose_};
        }

        // 导纳参数（需调整）
        std::array<double, 6> K = {1000.0, 1000.0, 1000.0, 50.0, 50.0, 50.0}; // 刚度
        std::array<double, 6> D = {100.0, 100.0, 100.0, 10.0, 10.0, 10.0};   // 阻尼
        double dt = 0.001; // 1ms

        // 计算速度
        std::array<double, 6> velocity;
        for (size_t i = 0; i < 6; i++) {
            velocity[i] = (current_force_torque[i] - K[i] * current_pose[i]) / D[i];
        }

        // 更新位姿
        std::array<double, 6> new_pose;
        for (size_t i = 0; i < 6; i++) {
            new_pose[i] = current_pose[i] + velocity[i] * dt;
        }

        RCLCPP_INFO(get_logger(), "Admittance: pose=[%f, %f, %f, %f, %f, %f], force=[%f, %f, %f, %f, %f, %f]",
                    new_pose[0], new_pose[1], new_pose[2], new_pose[3], new_pose[4], new_pose[5],
                    current_force_torque[0], current_force_torque[1], current_force_torque[2],
                    current_force_torque[3], current_force_torque[4], current_force_torque[5]);

        last_valid_pose_ = new_pose;
        if (has_new_force) {
            last_force_torque_ = current_force_torque;
        }
        return {new_pose};
    }

    void startRealtimeControl() {
        if (current_mode_ != ControlMode::Idle) {
            RCLCPP_WARN(get_logger(), "Cannot start Realtime mode, current mode: %s", modeToString(current_mode_).c_str());
            return;
        }
        current_mode_ = ControlMode::Realtime;
        RCLCPP_INFO(get_logger(), "Entering Realtime control mode");

        std::array<double, 7> target_pos {}, cur_pos {};
        std::error_code ec;
        robot_.getStateData(RtSupportedFields::jointPos_m, cur_pos, ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to get current positions: %s", ec.message().c_str());
            current_mode_ = ControlMode::Idle;
            return;
        }
        {
            std::lock_guard<std::mutex> lock(joint_positions_mutex_);
            target_pos = joint_queue_.empty() ? cur_pos : joint_queue_.front();
            if (!joint_queue_.empty()) {
                joint_queue_.pop_front();
            }
        }
        motion_controller_->MoveJ(0.5, cur_pos, target_pos, ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "MoveJ failed: %s", ec.message().c_str());
            current_mode_ = ControlMode::Idle;
            return;
        }

        {
            std::lock_guard<std::mutex> lock(joint_positions_mutex_);
            init_move_completed = true;
        }
        motion_controller_->setControlLoop(
            std::function<JointPosition()>(std::bind(&rt_RobotCtrlNode::rokae_callback, this)),
            0, true
        );
        motion_controller_->startMove(RtControllerMode::jointPosition, ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to start move: %s", ec.message().c_str());
            current_mode_ = ControlMode::Idle;
            return;
        }
        RCLCPP_INFO(get_logger(), "Control loop started.");
        control_thread_ = std::thread([this]() {
            try {
                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                RCLCPP_INFO(this->get_logger(), "Joint queue is %s before starting loop", joint_queue_.empty() ? "empty" : "not empty");
                if (!joint_queue_.empty()) {
                    const auto& front_point = joint_queue_.front();
                    RCLCPP_INFO(this->get_logger(), "First point in joint_queue_: [%f, %f, %f, %f, %f, %f, %f]",
                                front_point[0], front_point[1], front_point[2], front_point[3],
                                front_point[4], front_point[5], front_point[6]);
                }
                this->motion_controller_->startLoop(true);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "startLoop exception: %s", e.what());
                rclcpp::shutdown();
            }
        });
        control_loop_started_ = true;
    }

    void startComplianceControl() {
        if (current_mode_ != ControlMode::Idle) {
            RCLCPP_WARN(get_logger(), "Cannot start Compliance mode, current mode: %s", modeToString(current_mode_).c_str());
            return;
        }
        current_mode_ = ControlMode::Compliance;
        RCLCPP_INFO(get_logger(), "Entering Compliance control mode");

        std::error_code ec;
        robot_.setOperateMode(OperateMode::manual, ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to set manual mode: %s", ec.message().c_str());
            current_mode_ = ControlMode::Idle;
            return;
        }

        std::array<double, 6> current_pose {};
        robot_.getStateData(RtSupportedFields::cartesianPos, current_pose, ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to get initial cartesian pose: %s", ec.message().c_str());
            current_mode_ = ControlMode::Idle;
            return;
        }
        last_valid_pose_ = current_pose;

        motion_controller_->setControlLoop(
            std::function<CartesianPose()>(std::bind(&rt_RobotCtrlNode::admittance_callback, this)),
            0, true
        );
        motion_controller_->startMove(RtControllerMode::cartesianPosition, ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to start cartesian move: %s", ec.message().c_str());
            current_mode_ = ControlMode::Idle;
            return;
        }

        control_thread_ = std::thread([this]() {
            try {
                std::lock_guard<std::mutex> lock(force_torque_mutex_);
                RCLCPP_INFO(this->get_logger(), "Force queue is %s before starting loop", force_queue_.empty() ? "empty" : "not empty");
                if (!force_queue_.empty()) {
                    const auto& front_force = force_queue_.front();
                    RCLCPP_INFO(this->get_logger(), "First force in force_queue_: [Fx: %f, Fy: %f, Fz: %f, Tx: %f, Ty: %f, Tz: %f]",
                                front_force[0], front_force[1], front_force[2],
                                front_force[3], front_force[4], front_force[5]);
                }
                this->motion_controller_->startLoop(true);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "startLoop exception: %s", e.what());
                rclcpp::shutdown();
            }
        });
        control_loop_started_ = true;
        RCLCPP_INFO(get_logger(), "Admittance control loop started.");
    }

    void stopMotion() {
        std::error_code ec;
        if (control_loop_started_) {
            motion_controller_->stopLoop();
            if (control_thread_.joinable()) {
                control_thread_.join();
            }
            control_loop_started_ = false;
        }
        motion_controller_->stopMove(ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to stop move: %s", ec.message().c_str());
        }
        {
            std::lock_guard<std::mutex> lock1(joint_positions_mutex_);
            std::lock_guard<std::mutex> lock2(force_torque_mutex_);
            joint_queue_.clear();
            force_queue_.clear();
            last_force_torque_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            init_move_completed = false;
            init_joint_pos_set_ = false;
        }
        robot_.setOperateMode(OperateMode::automatic, ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to reset operate mode: %s", ec.message().c_str());
        }
        current_mode_ = ControlMode::Idle;
        RCLCPP_INFO(get_logger(), "Motion stopped, mode reset to Idle");
    }

    int kbhit() {
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
        if (ch != EOF) {
            ungetc(ch, stdin);
            return 1;
        }
        return 0;
    }

    void keyboard_input_thread() {
        while (rclcpp::ok()) {
            if (kbhit()) {
                char ch = getchar();
                switch (ch) {
                    case 'r':
                        startRealtimeControl();
                        break;
                    case 'f':
                        startComplianceControl();
                        break;
                    case 'q':
                        stopMotion();
                        break;
                    default:
                        RCLCPP_INFO(get_logger(), "Invalid key: %c, available: r (realtime), f (compliance), q (stop)", ch);
                        break;
                }
            }
            std::this_thread::sleep_for(10ms);
        }
    }

    std::string modeToString(ControlMode mode) {
        switch (mode) {
            case ControlMode::Idle: return "Idle";
            case ControlMode::Realtime: return "Realtime";
            case ControlMode::Compliance: return "Compliance";
            default: return "Unknown";
        }
    }

    rokae::xMateErProRobot robot_;
    std::shared_ptr<RtMotionControlCobot<7>> motion_controller_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_positions_sub_;
    rclcpp::Subscription<cust_msgs::msg::Stampfloat32array>::SharedPtr force_torque_sub_;
    const std::array<double, 7> zero_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 7> last_valid_command_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 6> last_valid_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 6> last_force_torque_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::mutex joint_positions_mutex_;
    std::mutex force_torque_mutex_;
    std::deque<std::array<double, 7>> joint_queue_;
    std::deque<std::array<double, 6>> force_queue_;
    const size_t max_queue_size_ = 5;
    bool init_joint_pos_set_ = false;
    bool init_move_completed = false;
    bool control_loop_started_ = false;
    std::thread control_thread_;
    std::thread keyboard_thread_;
    enum class ControlMode { Idle, Realtime, Compliance };
    ControlMode current_mode_ = ControlMode::Idle;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<rt_RobotCtrlNode>());
    } catch (const std::exception& e) {
        std::cerr << "Node failed: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}

// TODO

测试键盘三种模式运动控制没问题
测试力传感器数据正确回传
加入机械臂笛卡尔空间位置和速度控制代码
加入机械臂导纳控制代码
测试机械臂导纳控制实物实验

加入realsense包
基于ArUco码部署IBVS代码