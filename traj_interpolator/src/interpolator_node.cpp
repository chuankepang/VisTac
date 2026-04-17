#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>

// ros2 topic pub -r 10 /cmd_pose_vision std_msgs/msg/Float32MultiArray "{data: [1,0,0,0.2, 0,1,0,0, 0,0,1,0, 0,0,0,1]}"
// ros2 topic pub --once /interpolation_mode std_msgs/msg/Int32 "{data: 1}"

// 定义数据源枚举，用于状态机控制
enum class CommandSource {
    IDLE = 0,         // 静止模式（不接收任何新指令，保持当前位姿）
    VISION = 1,       // 听从视觉指令
    ADMITTANCE = 2    // 听从导纳/力控指令
};

class TrajInterpolator : public rclcpp::Node {
public:
    TrajInterpolator() : Node("traj_interpolator_node") {
        // --- 1. 模式订阅器：接收外层状态机指令 ---
        mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/interpolation_mode", 10, 
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                this->current_mode_ = static_cast<CommandSource>(msg->data);
                RCLCPP_INFO(this->get_logger(), "============= 模式切换至: %d =============", msg->data);
                // 切换模式时重置超时时间戳，防止刚切换就触发超时停机
                this->last_msg_time_ = this->now();
            });

        // --- 2. 视觉目标订阅器 ---
        vision_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/cmd_pose_vision", 10, 
            std::bind(&TrajInterpolator::visionCallback, this, std::placeholders::_1));
        
        // --- 3. 导纳目标订阅器 ---
        admittance_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/cmd_pose_admittance", 10, 
            std::bind(&TrajInterpolator::admittanceCallback, this, std::placeholders::_1));

        // --- 4. 发布器与定时器 ---
        smooth_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/target_cartpos", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&TrajInterpolator::updateLoop, this));

        // 初始状态设定
        current_mode_ = CommandSource::IDLE; 
        last_msg_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "通用轨迹插值节点已启动。当前模式: IDLE (等待状态机指令)");
    }

private:
    // --- 视觉频道专属回调 ---
    void visionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (current_mode_ == CommandSource::VISION) {
            processGoal(msg);
            last_msg_time_ = this->now(); // 更新最后收到消息的时间
        }
    }

    // --- 导纳频道专属回调 ---
    void admittanceCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (current_mode_ == CommandSource::ADMITTANCE) {
            processGoal(msg);
            last_msg_time_ = this->now(); // 更新最后收到消息的时间
        }
    }

    // --- 核心目标解析函数 ---
    void processGoal(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() != 16) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "收到无效矩阵大小: %zu", msg->data.size());
            return;
        }

        // float -> double 精度提升映射
        Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> target_mat_f(msg->data.data());
        Eigen::Matrix4d target_mat = target_mat_f.cast<double>();
        
        target_pos_ = target_mat.block<3, 1>(0, 3);
        target_quat_ = Eigen::Quaterniond(target_mat.block<3, 3>(0, 0));
        target_quat_.normalize();

        // 如果是第一帧数据，强制初始化当前位姿（防止原地乱飞）
        if (!initialized_) {
            current_pos_ = target_pos_;
            current_quat_ = target_quat_;
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "插值器成功获取首个基准位姿，开始正常插值！");
        }
    }

    // --- 500Hz 控制循环 ---
    void updateLoop() {
        if (!initialized_) return; // 没收到第一个有效点之前，静默等待

        // --- 安全机制：超时停机保护 (Timeout Check) ---
        // 如果不在 IDLE 模式，且超过 0.5 秒没收到新指令，强制目标原地锁死
        if (current_mode_ != CommandSource::IDLE) {
            auto dt = this->now() - last_msg_time_;
            if (dt.seconds() > 0.5) {
                target_pos_ = current_pos_;
                target_quat_ = current_quat_;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "【警告】上游指令超时断更 (>0.5s)！插值器已强制原地锁死防止碰撞。");
            }
        } else {
            // IDLE 模式下，强制目标位姿等于当前位姿，彻底锁死
            target_pos_ = current_pos_;
            target_quat_ = current_quat_;
        }

        // --- 1. 平移部分插值 (Linear) ---
        Eigen::Vector3d pos_diff = target_pos_ - current_pos_;
        double pos_dist = pos_diff.norm();
        if (pos_dist > max_linear_step_) {
            current_pos_ += (pos_diff / pos_dist) * max_linear_step_;
        } else {
            current_pos_ = target_pos_; // 追上了
        }

        // --- 2. 旋转部分插值 (Slerp) ---
        double angle = current_quat_.angularDistance(target_quat_);
        if (angle > max_angular_step_) {
            double t = max_angular_step_ / angle;
            current_quat_ = current_quat_.slerp(t, target_quat_);
        } else {
            current_quat_ = target_quat_;
        }
        current_quat_.normalize();

        // --- 优雅的打印调试 (每秒输出2次，不刷屏) ---
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
            "[Mode:%d] Current X: %.4f | Target X: %.4f | Dist: %.4f", 
            static_cast<int>(current_mode_), current_pos_.x(), target_pos_.x(), pos_dist);

        // --- 3. 封装并发布 ---
        auto out_msg = std_msgs::msg::Float32MultiArray();
        out_msg.data.resize(16);

        Eigen::Matrix4d res_mat = Eigen::Matrix4d::Identity();
        res_mat.block<3, 3>(0, 0) = current_quat_.toRotationMatrix();
        res_mat.block<3, 1>(0, 3) = current_pos_;

        // double -> float 降维映射输出
        Eigen::Matrix<float, 4, 4, Eigen::RowMajor> res_mat_f = res_mat.cast<float>();
        std::copy(res_mat_f.data(), res_mat_f.data() + 16, out_msg.data.begin());
        
        smooth_pub_->publish(out_msg);
    }

    // ROS 2 接口
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr vision_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr admittance_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr smooth_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 状态管理
    CommandSource current_mode_;
    rclcpp::Time last_msg_time_;
    bool initialized_ = false;

    // 运动学计算变量
    Eigen::Vector3d current_pos_, target_pos_;
    Eigen::Quaterniond current_quat_, target_quat_;

    // 插值限制参数 (决定机械臂最高速度)
    const double max_linear_step_ = 0.0002; // 2ms走0.4mm -> ~0.2 m/s
    const double max_angular_step_ = 0.001; // 2ms转0.002rad -> ~57.3 deg/s
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajInterpolator>());
    rclcpp::shutdown();
    return 0;
}