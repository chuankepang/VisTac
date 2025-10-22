#include <iostream>
#include <cmath>
#include <thread>
#include <mutex> // 用于线程同步
#include <array>
#include <termios.h>
#include <fcntl.h>
#include <deque>

// 错误：忘加了
#include <unistd.h>

#include "rokae_rt/robot.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"


using namespace std::chrono_literals;
using namespace rokae;

class rt_RobotCtrlNode :  public rclcpp::Node
{
public:
    rt_RobotCtrlNode() : Node("rt_robot_control_node")
    {
        // 初始化rokae机器人
        std::error_code ec;
        try
        {
            std::string robot_ip = "192.168.0.160";
            std::string local_ip = "192.168.0.100";
            robot_.connectToRobot(robot_ip, local_ip);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        robot_.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot_.setRtNetworkTolerance(20, ec);
        robot_.setOperateMode(OperateMode::automatic, ec);
        robot_.setPowerState(true,ec);
        
        //订阅节点
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        // 错误
        // .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        // .durability(rclcpp::DurabilityPolicy::Volatile)
        .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
        .deadline(rclcpp::Duration(1ms));

        joint_positions_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "sent_joints", qos, 
            std::bind(&rt_RobotCtrlNode::jointPositionCallback, this, std::placeholders::_1)
        );

        // filted_joints = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        //     "filted_joints", qos
        // );
        // 初始化关节位置
        try
        {
            RCLCPP_INFO(this->get_logger(), "Initializing robot joint positions to zero.");
            motion_controller_ = robot_.getRtMotionController().lock();
            robot_.startReceiveRobotState(std::chrono::milliseconds(1),{RtSupportedFields::jointPos_m});

            std::array<double, 7> cur_pos {};
            robot_.getStateData(RtSupportedFields::jointPos_m, cur_pos);
            
            std::cout << "Current joint positions: ";
            for (double value : cur_pos) {
                std::cout << value << " ";
            }
            std::cout << std::endl;

            PowerState state = robot_.powerState(ec);
            std::string state_str;
            switch (state)
            {
                case PowerState::on:
                    state_str = "上电";
                    break;
                case PowerState::off:
                    state_str = "下电";
                    break;
                case PowerState::unknown:
                    state_str = "未知";
                    break;
                case PowerState::estop:
                    state_str = "急停";
                    break;
                case PowerState::gstop:
                    state_str = "安全门打开";
                    break;
                default:
                    state_str = "无效状态";
                    break;
            }
            std::cout << "Robot power state: " << state_str << std::endl;
            
            
            motion_controller_->MoveJ(0.5,cur_pos,zero_pos);
            RCLCPP_INFO(this->get_logger(), "Robot joint positions initialized to zero.");
        }
        catch(std::exception &e)
        {
            std::cerr << e.what();
        }

        // TODO 控制循环代码
        // std::function<JointPosition()> rokae_callback = [&,this] ()
        // {
        //     bool is_ready_to_move = false;
        //     std::array<double,7> current_target_joint_pos_;
        //     {
        //         std::lock_guard<std::mutex> lock(joint_positions_mutex_);
        //         current_target_joint_pos_ = rec_joint_positions_;
        //         if(!init_joint_pos_set_ && has_received_joint_positions_ && init_move_completed) 
        //         {
        //             init_joint_pos_set_ = true;
        //         }
        //         is_ready_to_move = init_joint_pos_set_ ;
        //     }
        //     // else
        //     // {
        //     //     RCLCPP_INFO(this->get_logger(), "Received No init pos.Stay at zero position.");
        //     // }
        //     JointPosition cmd;
        //     // std::cout << init_joint_pos_set_<< std::endl;
        //     if(is_ready_to_move)
        //     {
        //         cmd.joints = std::vector<double>(current_target_joint_pos_.begin(), current_target_joint_pos_.end());
        //     }
        //     else
        //     {
        //         std::array<double, 7> current_pos {};
        //         robot_.getStateData(RtSupportedFields::jointPos_m, current_pos);
        //         cmd.joints = std::vector<double>(current_pos.begin(), current_pos.end());
        //     }
        //     return cmd;
        // };

        // motion_controller_->setControlLoop(rokae_callback);
        // motion_controller_->startMove(RtControllerMode::jointPosition);
        // control_thread_ = std::thread([this]() 
        // {
        //     this->motion_controller_->startLoop(true);
        // });

        keyboard_thread_ = std::thread([this]()
        {
            this->keyboard_input_thread();
        });
    };

    ~rt_RobotCtrlNode()
    {
        if(control_thread_.joinable())
        {
            motion_controller_->stopLoop();
            control_thread_.join();
            keyboard_thread_.join();
        }
        std::error_code ec;
        robot_.setPowerState(false, ec);
        std::cout<< "Robot power off." << std::endl;
    }

private:

    // joint_queue_是存储目标joint的队列
    // joint_queue_可能造成延迟，一是直接控制，取消队列；而是减小队列长度，减少缓冲
    void jointPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if(msg->data.size() == 7)
        {
            std::array<double, 7> new_positions;
            for (size_t i = 0; i < 7; i++)
            {
                new_positions[i] = static_cast<double>(msg->data[i]);
            }
            {
                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                if(joint_queue_.size() >= max_queue_size_) {
                    joint_queue_.pop_front();
                }
                joint_queue_.push_back(new_positions);
            }
        }
    }

    

    JointPosition rokae_callback()
    {
        bool is_ready_to_move = false;
        std::array<double,7> current_target_joint_pos_ {};
        {
            std::lock_guard<std::mutex> lock(joint_positions_mutex_);
            if(!joint_queue_.empty()) 
            {
                current_target_joint_pos_ = joint_queue_.front();
                joint_queue_.pop_front();
            }
            else
            {
                robot_.getStateData(RtSupportedFields::jointPos_m, current_target_joint_pos_);
            }

            if(!init_joint_pos_set_ && init_move_completed) 
            {
                init_joint_pos_set_ = true;
            }
            is_ready_to_move = init_joint_pos_set_ ;
        }

        JointPosition cmd;
        // std::cout << init_joint_pos_set_<< std::endl;
        if(is_ready_to_move)
        {
            cmd.joints = std::vector<double>(current_target_joint_pos_.begin(), current_target_joint_pos_.end());
        }
        else
        {
            std::array<double, 7> current_pos {};
            robot_.getStateData(RtSupportedFields::jointPos_m, current_pos);
            cmd.joints = std::vector<double>(current_pos.begin(), current_pos.end());
        }
        return cmd;
    }



    int kbhit(void)
    {
        struct termios oldt, newt;
        int ch;
        int oldf;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);
        if (ch != EOF)
        {
            ungetc(ch, stdin);
            return 1;
        }
        return 0;
    }

    void keyboard_input_thread()
    {
        while (rclcpp::ok())  
        {
            if (kbhit())
            {
                char ch = getchar();
                switch (ch)
                {

                        break;
                    case 'c':
                        if (!control_loop_started_) 
                        {
                            std::array<double, 7> target_pos {};
                            {
                                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                                target_pos = joint_queue_.empty() ? zero_pos : joint_queue_.front();
                                joint_queue_.clear();
                            }

                            std::array<double, 7> cur_pos {};
                            robot_.getStateData(RtSupportedFields::jointPos_m, cur_pos);
                            std::cout << "Current target positions: ";
                            for (double value : target_pos) {
                                std::cout << value << " ";
                            }
                            std::cout << std::endl;
                            motion_controller_->MoveJ(0.5,cur_pos,target_pos);

                            {
                                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                                init_move_completed = true;
                            }
                            motion_controller_->setControlLoop(
                                std::function<JointPosition()>(std::bind(&rt_RobotCtrlNode::rokae_callback, this)),
                                0,
                                true
                            );
                            motion_controller_->startMove(RtControllerMode::jointPosition);
                            RCLCPP_INFO(this->get_logger(), "Control loop started.");
                            control_thread_ = std::thread([this]() 
                            {
                                // try 
                                // {
                                //     this->motion_controller_->startLoop(true);
                                // } catch (const std::exception& e) 
                                // {
                                //     RCLCPP_ERROR(this->get_logger(), "startLoop exception: %s", e.what());
                                // }

                                try {
                                        std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                                        if (joint_queue_.empty()) {
                                            RCLCPP_INFO(this->get_logger(), "joint_queue_ is empty before starting real-time control loop");
                                        } else {
                                            RCLCPP_INFO(this->get_logger(), "joint_queue_ is not empty, size: %zu", joint_queue_.size());
                                            // 可选：打印第一个点
                                            const auto& front_point = joint_queue_.front();
                                            RCLCPP_INFO(this->get_logger(), "First point in joint_queue_: [%f, %f, %f, %f, %f, %f, %f]",
                                                        front_point[0], front_point[1], front_point[2], front_point[3],
                                                        front_point[4], front_point[5], front_point[6]);
                                        }
                                        RCLCPP_INFO(this->get_logger(), "Starting real-time control loop");
                                        this->motion_controller_->startLoop(true);
                                    } catch (const std::exception& e) {
                                        RCLCPP_ERROR(this->get_logger(), "startLoop exception: %s", e.what());
                                        rclcpp::shutdown();
                                    }

                            });
                            control_loop_started_ = true;
                        }
                        break;
                    case 'q':
                        std::cout << "Exiting..." << std::endl;
                        rclcpp::shutdown();  
                        return;
                    default:
                        break;
                }
            }
            std::this_thread::sleep_for(10ms);
        }
    }

    rokae::xMateErProRobot robot_;

    std::shared_ptr<RtMotionControlCobot<7>> motion_controller_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_positions_sub_;

    // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr filted_joints;
    
    const std::array<double, 7> zero_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //线程共享数据
    std::mutex joint_positions_mutex_;
    std::deque<std::array<double,7>> joint_queue_;
    const size_t max_queue_size_ = 100; // 定长队列大小


    bool init_joint_pos_set_ = false;
    bool init_move_completed = false;
    bool control_loop_started_ = false;

    std::thread control_thread_;
    std::thread keyboard_thread_; 

};


// rt_RobotCtrlNode类作为核心节点，首先在构造函数中初始化ROS 2节点，名为"rt_robot_control_node"。
// 随后，它尝试连接到Rokae机械臂，使用指定的机器人IP地址(192.168.0.160)和本地IP地址(192.168.0.100)调用robot_.connectToRobot函数，如果连接失败则捕获异常、记录错误日志并关闭节点。
// 连接成功后，配置机械臂为实时命令模式(RtCommand)、设置网络容忍度为20毫秒、切换到自动操作模式，并上电启用电源状态。接下来，节点获取当前关节位置，执行MoveJ命令将机械臂移动到零位初始化姿态，同时启动1毫秒周期的机器人状态接收(聚焦关节位置数据)。
// 然后，节点设置实时控制循环回调rokae_callback，并启动关节位置控制模式，通过独立控制线程运行startLoop来周期性调用rokae_callback生成并下发JointPosition命令。
// 同时，启动并行键盘输入线程keyboard_input_thread，每100毫秒检测键盘按键，如果按下'q'则关闭节点，按下'f'则调用force_srv服务处理力传感器数据。
// 在运行过程中，/sent_joints话题的消息触发jointPositionCallback回调函数，将接收的7个关节位置数据存储到joint_queue_队列中(队列满时覆盖最旧数据)。
// rokae_callback在控制循环中从队列取出目标位置(或当前位置)，根据初始化状态决定是否准备移动(is_ready_to_move)，生成JointPosition命令下发给底层控制器，实现机械臂的实时跟踪。
// 如果通信中断或按键停止，节点关闭，运动控制终止。

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rt_RobotCtrlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}