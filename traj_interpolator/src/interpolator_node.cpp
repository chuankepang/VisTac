#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>
#include <algorithm>

class TrajInterpolator : public rclcpp::Node {
public:
    TrajInterpolator() : Node("traj_interpolator_node") {
        vision_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/cmd_pose_vision", 10, std::bind(&TrajInterpolator::goalCallback, this, std::placeholders::_1));
        
        admittance_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/cmd_pose_admittance", 10, std::bind(&TrajInterpolator::goalCallback, this, std::placeholders::_1));

        smooth_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/target_cartpos", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&TrajInterpolator::updateLoop, this));

        RCLCPP_INFO(this->get_logger(), "通用轨迹插值节点已启动，修复了 float/double 类型匹配问题。");
    }

private:
    void goalCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() != 16) {
            RCLCPP_WARN(this->get_logger(), "收到无效矩阵大小: %zu", msg->data.size());
            return;
        }

        // --- 修复点 1：先映射为 float 矩阵，再转换 (cast) 为 double ---
        Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> target_mat_f(msg->data.data());
        Eigen::Matrix4d target_mat = target_mat_f.cast<double>();
        
        target_pos_ = target_mat.block<3, 1>(0, 3);
        target_quat_ = Eigen::Quaterniond(target_mat.block<3, 3>(0, 0));
        target_quat_.normalize();

        if (!initialized_) {
            current_pos_ = target_pos_;
            current_quat_ = target_quat_;
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "插值器初始位姿已设定。");
        }
    }

    void updateLoop() {
        if (!initialized_) return;

        // 平移插值
        Eigen::Vector3d pos_diff = target_pos_ - current_pos_;
        double pos_dist = pos_diff.norm();
        if (pos_dist > max_linear_step_) {
            current_pos_ += (pos_diff / pos_dist) * max_linear_step_;
        } else {
            current_pos_ = target_pos_;
        }

        // 旋转插值 (Slerp)
        double angle = current_quat_.angularDistance(target_quat_);
        if (angle > max_angular_step_) {
            double t = max_angular_step_ / angle;
            current_quat_ = current_quat_.slerp(t, target_quat_);
        } else {
            current_quat_ = target_quat_;
        }
        current_quat_.normalize();

        // 封装并发布
        auto out_msg = std_msgs::msg::Float32MultiArray();
        out_msg.data.resize(16);

        Eigen::Matrix4d res_mat = Eigen::Matrix4d::Identity();
        res_mat.block<3, 3>(0, 0) = current_quat_.toRotationMatrix();
        res_mat.block<3, 1>(0, 3) = current_pos_;

        // --- 修复点 2：将计算结果转回 float 矩阵，再映射到 msg 容器中 ---
        Eigen::Matrix<float, 4, 4, Eigen::RowMajor> res_mat_f = res_mat.cast<float>();
        
        // 使用 std::copy 或者是重新映射，这里推荐 std::copy 更稳健
        std::copy(res_mat_f.data(), res_mat_f.data() + 16, out_msg.data.begin());
        
        smooth_pub_->publish(out_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr vision_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr admittance_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr smooth_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    Eigen::Vector3d current_pos_, target_pos_;
    Eigen::Quaterniond current_quat_, target_quat_;
    bool initialized_ = false;

    const double max_linear_step_ = 0.0004; 
    const double max_angular_step_ = 0.002; 
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajInterpolator>());
    rclcpp::shutdown();
    return 0;
}