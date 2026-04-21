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
    
    // 手眼标定矩阵：从法兰盘(Flange)到相机(Camera)的变换
    // 固定不变，在构造函数或 init 中初始化
    Eigen::Matrix4d flange_T_camera_;

    // ... 其他变量 ...
    Eigen::Matrix4d latest_camera_T_object_ = Eigen::Matrix4d::Identity();
    bool vision_received_ = false; // 标记是否收到过视觉数据
    std::mutex vision_data_mutex_; // 专门保护视觉原始数据的锁

    /**
    * @brief 计算物体在基坐标系下的位姿 (base_T_object)
    * @param raw_flange_pose 机械臂实时反馈的 16 维数组 (RowMajor)
    * @param cam_T_obj 视觉系统输出的目标位姿 (Eigen 形式)
    * @return Eigen::Matrix4d 转换后的基座坐标系位姿
    */
    Eigen::Matrix4d computeTargetInBase(const std::array<double, 16>& raw_flange_pose, 
                                        const Eigen::Matrix4d& cam_T_obj) 
    {
        // 1. 将机械臂返回的数组映射为 Eigen 矩阵 (base_T_flange)
        // Rokae 的 tcpPose_m 通常是 RowMajor 存储的 4x4 齐次矩阵
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> base_T_flange(raw_flange_pose.data());

        // 2. 执行链式相乘: T_base_obj = T_base_flange * T_flange_cam * T_cam_obj
        Eigen::Matrix4d base_T_object = base_T_flange * flange_T_camera_ * cam_T_obj;

        // 3. 数值稳定性处理：强制正交化旋转矩阵部分
        // 经过多次矩阵乘法，旋转部分可能不再是严格的单位正交阵，这会导致底层控制器报错
        Eigen::Matrix3d R = base_T_object.block<3, 3>(0, 0);
        Eigen::Quaterniond q(R);
        q.normalize(); // 归一化四元数
        base_T_object.block<3, 3>(0, 0) = q.toRotationMatrix();

        return base_T_object;
    }

    /**
     * @brief 将 Eigen 4x4 齐次变换矩阵转为 Rokae 的 CartesianPosition
     */
    rokae::CartesianPosition matrixToCartPos(const Eigen::Matrix4d& mat) {
        rokae::CartesianPosition cp;
        
        // 1. 提取平移部分 (X, Y, Z)，Rokae 单位是米 (m)
        // Frame 类内部有 trans 成员
        cp.trans = { mat(0, 3), mat(1, 3), mat(2, 3) };
        
        // 2. 提取旋转部分并转换为欧拉角 RPY (Roll, Pitch, Yaw)
        // Rokae 的 RPY 通常对应绕 X-Y-Z 轴的旋转，单位是弧度 (rad)
        // Eigen 的 eulerAngles(0, 1, 2) 提取顺序即为 X-Y-Z
        Eigen::Vector3d euler = mat.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
        cp.rpy = { euler[0], euler[1], euler[2] };
        
        // （可选）如果你的机械臂是 7 轴的，可能需要配置 elbow 或 confData
        // cp.hasElbow = false; 

        return cp;
    }

    // // --- 回调函数 1：模型指令接收 (低频 5Hz) ---
    // void cartPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    // {
    //     if (msg->data.size() == 16) {
    //         Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> f_mat(msg->data.data());
    //         Eigen::Matrix4d d_mat = f_mat.cast<double>();
            
    //         Eigen::Vector3d new_p = d_mat.block<3, 1>(0, 3);
    //         Eigen::Quaterniond new_q = Eigen::Quaterniond(d_mat.block<3, 3>(0, 0));
    //         new_q.normalize();

    //         std::lock_guard<std::mutex> lock(target_mutex_);
            
    //         // 核心灵魂：强行从当前的“微观滤波位置”作为新周期的起点
    //         // 无论上一个动作有没有完成，无论网络丢了多少帧，全盘接纳当前状态！
    //         start_p_ = curr_p_;
    //         start_q_ = curr_q_;
            
    //         end_p_ = new_p;
    //         end_q_ = new_q;
            
    //         // 四元数防绕远路处理
    //         if (start_q_.dot(end_q_) < 0.0) {
    //             end_q_.coeffs() *= -1.0;
    //         }
            
    //         // 刷新时间戳
    //         start_time_ = std::chrono::steady_clock::now();
    //         end_time_ = start_time_ + interp_duration_;
    //     }
    // }

    void cartPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() == 16) {
            // 1. 将收到的 Float32MultiArray 映射为相机坐标系下的目标位姿 camera_T_object
            Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> f_mat(msg->data.data());
            Eigen::Matrix4d cam_T_obj = f_mat.cast<double>();

            // 【新增】保存一份原始视觉快照，供离线阶段（keyboard_thread）的 TODO 使用
            {
                std::lock_guard<std::mutex> lock(vision_data_mutex_); // 建议新增一个锁专门护航原始数据
                latest_camera_T_object_ = cam_T_obj;
                vision_received_ = true;
            }

            // 2. 获取当前这一时刻机械臂的真实位姿 (base_T_flange)
            // 注意：这是实时闭环的关键，确保转换基于最新的机器人状态
            std::array<double, 16> current_raw_flange;
            robot_.getStateData(RtSupportedFields::tcpPose_m, current_raw_flange);

            // 3. 调用我们写好的 Private 函数进行坐标系转换
            // 计算公式：base_T_object = base_T_flange * flange_T_camera * camera_T_object
            Eigen::Matrix4d target_in_base = computeTargetInBase(current_raw_flange, cam_T_obj);

            // 4. 提取转换后的位置和姿态
            Eigen::Vector3d new_p = target_in_base.block<3, 1>(0, 3);
            Eigen::Quaterniond new_q = Eigen::Quaterniond(target_in_base.block<3, 3>(0, 0));
            new_q.normalize();

            // 5. 进入原有的“灵魂”插值逻辑
            std::lock_guard<std::mutex> lock(target_mutex_);
            
            // 强行从当前的“微观滤波位置”作为新周期的起点，接纳当前状态
            start_p_ = curr_p_;
            start_q_ = curr_q_;
            
            end_p_ = new_p; // 现在的 end_p_ 已经是 Base 坐标系下的绝对坐标了
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

                    if (!vision_received_) {
                            RCLCPP_WARN(this->get_logger(), "Waiting for vision data...");
                            continue;
                        }

                    // 1. 获取最新视觉数据快照
                    Eigen::Matrix4d snap_cam_T_obj;
                    {
                        std::lock_guard<std::mutex> lock(vision_data_mutex_);
                        snap_cam_T_obj = latest_camera_T_object_;
                    }

                    // 1. 获取当前机械臂位姿
                    std::array<double, 16> current_raw_pose;
                    robot_.getStateData(RtSupportedFields::tcpPose_m, current_raw_pose);
                    Eigen::Matrix4d target_in_base = computeTargetInBase(current_raw_pose, snap_cam_T_obj);

                    // 3. 计算预抓取点 (相对于目标物体 Z 轴向上退 10cm)
                    // 构造一个在物体本地坐标系下的偏移
                    Eigen::Matrix4d T_offset = Eigen::Matrix4d::Identity();
                    T_offset(2, 3) = -0.1; // 沿物体 Z 轴负方向(通常是垂直上方)移动 10cm

                    // 预抓取位姿 = 物体位姿 * 偏移
                    Eigen::Matrix4d pre_grasp_pose = target_in_base * T_offset;

                    // 1. 提取当前的真实位姿矩阵（将 array16 转为 Eigen）
                    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> current_mat(current_raw_pose.data());

                    // 2. 使用我们的转换函数创建 Rokae 指令结构体
                    rokae::CartesianPosition start_cmd = matrixToCartPos(current_mat);
                    rokae::CartesianPosition target_cmd = matrixToCartPos(pre_grasp_pose);

                    // --- 4. 执行 Phase 1：大范围阻塞式移动 ---
                    RCLCPP_INFO(this->get_logger(), "Phase 1: Moving to pre-grasp point...");
                    try {
                        // speed 设置为 0.5 (50% 速度)
                        // 此时传入的 start_cmd 和 target_cmd 都是标准的 CartesianPosition 格式
                        motion_controller_->MoveL(0.5, start_cmd, target_cmd); 
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Phase 1 Motion failed: %s", e.what());
                        continue; // 运动失败则不进入 Phase 2，继续等待下一次 'c'
                    }

                    RCLCPP_INFO(this->get_logger(), "Phase 1 Complete. Ready for RT Servoing.");

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