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
        
        //订阅节点 设置通信节点接受情况
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        // 1. 改变可靠性：BestEffort 通常用于实时控制，但如果发送端是 Reliable，
        //    最好也改成 Reliable 以确保在调试时能对齐。
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        // 2. 改变耐久性：改为 Volatile 即可
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


        // 笛卡尔空间话题消息，cartPositionCallback
        cart_positions_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "target_cartpos", qos, 
            std::bind(&rt_RobotCtrlNode::cartPositionCallback, this, std::placeholders::_1)
        );

        // filted_joints = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        //     "filted_joints", qos
        // );
        // 初始化关节位置
        try
        {
            RCLCPP_INFO(this->get_logger(), "Initializing robot joint positions to zero.");
            motion_controller_ = robot_.getRtMotionController().lock();
            robot_.startReceiveRobotState(std::chrono::milliseconds(1),{RtSupportedFields::jointPos_m, RtSupportedFields::tcpPose_m});

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
            
            // --- 新增代码：打印 zero_pos 对应的笛卡尔矩阵 ---
            std::array<double, 16> cur_cart_pos{};
            robot_.getStateData(RtSupportedFields::tcpPose_m, cur_cart_pos); 

            printf("\n[Zero Position Cartesian Matrix (4x4 Row-Major)]:\n");
            for (int i = 0; i < 4; i++) {
                printf("  [ %8.4f, %8.4f, %8.4f, %8.4f ]\n", 
                    cur_cart_pos[i*4], cur_cart_pos[i*4+1], cur_cart_pos[i*4+2], cur_cart_pos[i*4+3]);
            }
            printf("\n");
            
            motion_controller_->MoveJ(0.5, cur_pos, zero_pos);
            RCLCPP_INFO(this->get_logger(), "Robot joint positions initialized to zero.");

            // --- 新增代码：打印 zero_pos 对应的笛卡尔矩阵 ---
            std::array<double, 16> zero_cart_pos{};
            robot_.getStateData(RtSupportedFields::tcpPose_m, zero_cart_pos); 

            printf("\n[Zero Position Cartesian Matrix (4x4 Row-Major)]:\n");
            for (int i = 0; i < 4; i++) {
                printf("  [ %8.4f, %8.4f, %8.4f, %8.4f ]\n", 
                    zero_cart_pos[i*4], zero_cart_pos[i*4+1], zero_cart_pos[i*4+2], zero_cart_pos[i*4+3]);
            }
            printf("\n");
            // ------------------------------------------

        }
        catch(std::exception &e)
        {
            std::cerr << e.what();
        }


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



    // cart_queue_是存储目标joint的队列
    // cart_queue_可能造成延迟，一是直接控制，取消队列；而是减小队列长度，减少缓冲
    /**
    * @brief 笛卡尔位姿订阅回调
    * 消息格式约定 msg->data: 16个元素组成的行优先齐次变换矩阵 (4x4 Matrix)
    * 包含：旋转矩阵 (3x3) + 平移向量 (3x1) + 最后一行 [0,0,0,1]
    */
    void cartPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // 必须严格校验 16 个数，少一个都会导致机器人解算失败
        if(msg->data.size() == 16)
        {
            std::array<double, 16> new_matrix;
            for (size_t i = 0; i < 16; i++)
            {
                new_matrix[i] = static_cast<double>(msg->data[i]);
            }

            {
                // 使用笛卡尔专用的互斥锁
                std::lock_guard<std::mutex> lock(cart_positions_mutex_);

                // 保持实时性：如果队列满了，踢掉旧指令，放入最新矩阵
                if(cart_queue_.size() >= max_queue_size_) 
                {
                    cart_queue_.pop_front();
                }
                
                cart_queue_.push_back(new_matrix);
            }
        }
        else
        {
            // 报错提醒：发送端的数据长度不对
            RCLCPP_WARN(this->get_logger(), "Invalid Matrix size! Expected 16 (4x4), but received %zu", msg->data.size());
        }
    }
    


    CartesianPosition rokae_callback()
    {
        CartesianPosition cmd;
        // 使用临时变量，减少锁占用的时间
        std::array<double, 16> target_pos;
        bool from_queue = false;

        {
            std::lock_guard<std::mutex> lock(cart_positions_mutex_);
            
            if(!cart_queue_.empty()) 
            {
                // 1. 队列有数据，取出新数据并更新“最后有效位置”
                target_pos = cart_queue_.front();
                cart_queue_.pop_front();
                last_valid_cart_pos_ = target_pos; 
                from_queue = true; // 标记数据来自队列
            }
            else
            {
                // 2. 队列为空，直接发送“最后一次有效的位置”
                // 这保证了如果没有新话题，机械臂会稳稳地停在原地，不会有任何跳变
                target_pos = last_valid_cart_pos_;
                from_queue = false; // 标记数据来自上一帧备份
            }
        }

    // --- 调试打印开始 ---
        static int print_count = 0;
        if (print_count++ % 500 == 0) // 每 500 次循环（约 0.5 秒）打印一次
        {
            if (from_queue) {
                printf("[Real-Time] Data from QUEUE. Target Z: %.4f\n", target_pos[11]);
            } else {
                printf("[Real-Time] Queue EMPTY. Holding Z: %.4f\n", target_pos[11]);
            }
            
            // 如果想看完整矩阵（可选）
            /*
            printf("Matrix: [%.3f, %.3f, %.3f, %.3f | ... | %.3f]\n", 
                    target_pos[0], target_pos[1], target_pos[2], target_pos[3], target_pos[15]);
            */
        }
        // --- 调试打印结束 ---

        // 赋值给 SDK 结构体
        cmd.pos = target_pos;
        // static double offset = 0;
        // offset += 0.0001; // 每毫秒移动 0.1mm
        // cmd.pos[11] += offset; // 在 Z 轴上做简易漂移
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
                            // 1. 启动前，先获取当前的真实物理位置
                            std::array<double, 16> cur_pos {};
                            robot_.getStateData(RtSupportedFields::tcpPose_m, cur_pos);

                            // 2. 关键：初始化 last_valid_cart_pos_ 并清空旧队列
                            {
                                std::lock_guard<std::mutex> lock(cart_positions_mutex_);
                                last_valid_cart_pos_ = cur_pos; // 保证 callback 启动第一帧发出的就是当前位置
                                while(!cart_queue_.empty()) cart_queue_.pop_front(); // 清空启动前积压的过时话题
                                init_move_completed = true;
                            }

                            std::cout << "Starting Control loop. Initial Z: " << cur_pos[11] << std::endl;

                            // 3. 配置回调和模式
                            motion_controller_->setControlLoop(
                                std::function<CartesianPosition()>(std::bind(&rt_RobotCtrlNode::rokae_callback, this)),
                                0,
                                true
                            );
                            motion_controller_->startMove(RtControllerMode::cartesianPosition);
                            
                            RCLCPP_INFO(this->get_logger(), "Control mode enabled.");

                            // 4. 启动实时线程
                            control_thread_ = std::thread([this]() 
                            {
                                try {
                                    // 注意：这里已经没有锁了，确保 startLoop 不被锁卡死
                                    RCLCPP_INFO(this->get_logger(), "Entering startLoop...");
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
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cart_positions_sub_;

    // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr filted_joints;
    
    const std::array<double, 7> zero_pos = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};
    //线程共享数据
    std::mutex cart_positions_mutex_;
    std::deque<std::array<double,16>> cart_queue_;  // 16个元素（4x4齐次变换矩阵）
    const size_t max_queue_size_ = 100; // 定长队列大小

    std::array<double, 16> last_valid_cart_pos_;


    bool init_joint_pos_set_ = false;
    bool init_cart_pos_set_ = false;
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