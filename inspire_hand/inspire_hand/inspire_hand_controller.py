#!/usr/bin/env python3
"""
灵巧手独立控制节点
支持：捏取、释放、预定义动作、扩展动作库
"""

# ros2 topic pub --once /hand_action std_msgs/msg/String "{data: 'pinch'}"

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
from cust_msgs.msg import Stampint32array
import serial
import time
from typing import List, Dict, Callable

# 导入底层驱动
from .ins_read import write_data_6, read_data_6, forceClb, init_pos


class InspireHandController(Node):
    """
    灵巧手控制节点
    - 管理硬件连接和初始化
    - 提供统一的控制接口
    - 支持预定义动作和自定义动作
    """
    
    def __init__(self):
        super().__init__('inspire_hand_controller')
        
        # ========== 1. 硬件配置 ==========
        self.port = '/dev/ttyUSB0'  # 根据实际情况修改
        self.baudrate = 115200
        self.hand_id = 1
        
        # ========== 2. 动作库（可扩展）==========
        # 基础动作（6个手指的角度值 0-1000）
        self.actions = {
            # 小拇指、无名指、中指、食指、大拇指2
            # 基础动作
            'reset':    [0, 0, 0, 0, 0, 0],           # 复位/初始位置
            'open':     [1000, 1000, 1000, 1000, 1000, 1000],  # 完全张开
            'close':    [0, 0, 0, 0, 0, 0],           # 完全握拳
            
            # 捏取相关动作（你的核心需求）
            'pinch':    [1000, 0, 500, 500, 0, 0],    # 捏取（拇指+食指）
            'pinch_light': [0, 0, 0, 800, 600, 600],  # 轻捏
            'pinch_strong': [0, 0, 0, 1000, 1000, 1000],  # 强力捏取
            
            # 释放相关动作
            'release':  [1000, 1000, 1000, 1000, 800, 800],   # 释放
            'release_fast': [1000, 1000, 1000, 1000, 1000, 1000],  # 快速释放
            
            # 全抓握（拓展）
            'power_grasp':   [0, 0, 0, 0, 0, 0],      # 全握拳
            'hook_grasp':    [800, 800, 800, 800, 0, 0],   # 钩状抓握
            'precision_grasp': [500, 500, 1000, 1000, 300, 300],  # 精确抓握
            
            # 特殊手势（拓展）
            'victory':   [0, 0, 1000, 1000, 0, 0],    # 胜利手势
            'point':     [0, 0, 1000, 0, 0, 0],      # 指点手势
            'ok':        [0, 0, 0, 1000, 500, 500],  # OK手势
        }
        
        # 动作参数（可配置）
        self.action_params = {
            'grasp_speed': 1000,      # 抓取速度
            'grasp_force': 800,       # 抓取力阈值
            'hold_time': 2.0,         # 保持时间（秒）
        }
        
        # ========== 3. 状态变量 ==========
        self.current_angles = init_pos  # 当前角度
        self.current_force = [0, 0, 0, 0, 0, 0]  # 当前力值
        self.is_ready = False          # 是否就绪
        self.is_grasping = False       # 是否正在抓取
        self.current_action = None     # 当前执行的动作
        
        # ========== 4. ROS2接口 ==========
        # 发布状态
        self.state_pub = self.create_publisher(
            Stampint32array, 
            'hand_states', 
            10)
        
        # 订阅控制指令（接收外部命令）
        self.control_sub = self.create_subscription(
            Int32MultiArray,
            'hand_control',      # 统一控制话题
            self.control_callback,
            1)
        
        # 订阅动作指令（接收动作名称）
        self.action_sub = self.create_subscription(
            String,              # 需要导入 String
            'hand_action',       # 动作名称话题
            self.action_callback,
            1)
        
        # ========== 5. 初始化硬件 ==========
        self.ser = self.init_hardware()
        
        # ========== 6. 状态定时器 ==========
        self.state_timer = self.create_timer(0.05, self.publish_state)  # 20Hz
        
        self.get_logger().info("灵巧手控制器已启动")
        self.get_logger().info("可用动作: " + ", ".join(self.actions.keys()))
    
    # ==================== 硬件初始化 ====================
    
    def init_hardware(self):
        """初始化灵巧手硬件"""
        try:
            # 1. 打开串口
            ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=0.1
            )
            self.get_logger().info(f"串口打开成功: {self.port}")
            
            # 2. 力校准
            self.get_logger().info("力校准中，请确保灵巧手静止...")
            forceClb(ser, self.hand_id)
            self.get_logger().info("力校准完成")
            
            # 3. 设置力阈值
            force_threshold = [self.action_params['grasp_force']] * 6
            write_data_6(ser, self.hand_id, 'forceSet', force_threshold)
            self.get_logger().info(f"力阈值设置为: {force_threshold}")
            
            # 4. 设置速度
            speed = [self.action_params['grasp_speed']] * 6
            write_data_6(ser, self.hand_id, 'speedSet', speed)
            self.get_logger().info(f"速度设置为: {speed}")
            
            # 5. 复位到初始位置
            write_data_6(ser, self.hand_id, 'angleSet', init_pos)
            time.sleep(0.5)
            
            self.is_ready = True
            self.get_logger().info("灵巧手初始化完成！")
            return ser
            
        except Exception as e:
            self.get_logger().error(f"硬件初始化失败: {e}")
            return None
    
    # ==================== 控制接口 ====================
    
    def control_callback(self, msg):
        """
        控制话题回调
        接收 Int32MultiArray 格式的控制指令
        """
        if not self.is_ready:
            self.get_logger().warn("灵巧手未就绪，忽略控制指令")
            return
        
        if len(msg.data) != 6:
            self.get_logger().error(f"无效的角度数组长度: {len(msg.data)}")
            return
        
        # 安全检查
        if not self.is_safe_angle(msg.data):
            self.get_logger().warn("不安全的角度值，拒绝执行")
            return
        
        # 执行控制
        self.execute_angles(msg.data)
        self.get_logger().debug(f"执行角度: {msg.data}")
    
    def action_callback(self, msg):
        """
        动作话题回调
        接收动作名称字符串
        """
        action_name = msg.data
        self.execute_action(action_name)
    
    def execute_action(self, action_name: str):
        """
        执行预定义动作
        
        参数:
            action_name: 动作名称，如 'pinch', 'release', 'open' 等
        """
        if not self.is_ready:
            self.get_logger().warn("灵巧手未就绪，无法执行动作")
            return False
        
        if action_name not in self.actions:
            self.get_logger().error(f"未定义的动作: {action_name}")
            self.get_logger().info(f"可用动作: {list(self.actions.keys())}")
            return False
        
        angles = self.actions[action_name]
        self.get_logger().info(f"执行动作: {action_name} -> {angles}")
        
        # 执行动作
        self.execute_angles(angles)
        self.current_action = action_name
        
        return True
    
    def execute_angles(self, angles: List[int]):
        """
        执行角度控制（核心控制函数）
        
        参数:
            angles: 6个手指的角度 [0-1000]
        """
        if self.ser is None:
            self.get_logger().error("串口未连接")
            return
        
        try:
            # 发送控制指令
            write_data_6(self.ser, self.hand_id, 'angleSet', angles)
            self.current_angles = angles
            
            # 可选：等待动作完成（简单延时）
            # 实际应用中可以根据力反馈判断
            
        except Exception as e:
            self.get_logger().error(f"执行角度控制失败: {e}")
    
    # ==================== 高级抓取函数 ====================
    
    def pinch_grasp(self, hold_time: float = None):
        """
        捏取动作（带时间控制）
        
        参数:
            hold_time: 保持时间（秒），None表示持续保持
        """
        if hold_time is None:
            # 持续捏取
            self.execute_action('pinch')
            self.is_grasping = True
            self.get_logger().info("捏取并保持")
        else:
            # 捏取-保持-释放
            self.execute_action('pinch')
            self.get_logger().info(f"捏取，保持 {hold_time} 秒")
            time.sleep(hold_time)
            self.execute_action('release')
            self.get_logger().info("释放")
    
    def force_controlled_grasp(self, target_force: int = 500, max_time: float = 3.0):
        """
        力控抓取（带力反馈）
        
        参数:
            target_force: 目标力阈值
            max_time: 最大抓取时间
        """
        self.get_logger().info(f"开始力控抓取，目标力: {target_force}")
        
        start_time = time.time()
        current_force = self.read_force()
        
        # 逐渐增加抓取力
        for angle in range(0, 1001, 50):
            if time.time() - start_time > max_time:
                self.get_logger().warn("抓取超时")
                break
            
            # 发送角度
            angles = [angle, angle, angle, angle, angle, angle]
            self.execute_angles(angles)
            time.sleep(0.05)
            
            # 读取力反馈
            current_force = self.read_force()
            if max(current_force) > target_force:
                self.get_logger().info(f"达到目标力值: {max(current_force)}")
                break
        
        self.get_logger().info("力控抓取完成")
        return max(current_force) > target_force
    
    # ==================== 状态读取 ====================
    
    def read_force(self):
        """读取当前力值"""
        if self.ser is None:
            return [0, 0, 0, 0, 0, 0]
        
        try:
            force = read_data_6(self.ser, self.hand_id, 'forceAct')
            if force:
                # 转换有符号数
                force = [(f - 65536) if f > 10000 else f for f in force]
                self.current_force = force
                return force
        except Exception as e:
            self.get_logger().debug(f"读取力值失败: {e}")
        
        return self.current_force
    
    def read_angles(self):
        """读取当前实际角度"""
        if self.ser is None:
            return self.current_angles
        
        try:
            angles = read_data_6(self.ser, self.hand_id, 'angleAct')
            if angles:
                self.current_angles = angles
                return angles
        except Exception as e:
            self.get_logger().debug(f"读取角度失败: {e}")
        
        return self.current_angles
    
    # ==================== 安全函数 ====================
    
    def is_safe_angle(self, angles: List[int]) -> bool:
        """检查角度值是否安全"""
        for angle in angles:
            if not 0 <= angle <= 1000:
                self.get_logger().error(f"角度值超出范围: {angle}")
                return False
        return True
    
    def emergency_stop(self):
        """紧急停止"""
        self.get_logger().warn("紧急停止！")
        self.execute_angles([0, 0, 0, 0, 0, 0])  # 握拳停止
        self.is_grasping = False
    
    # ==================== 状态发布 ====================
    
    def publish_state(self):
        """发布当前状态"""
        if not self.is_ready:
            return
        
        # 更新当前状态
        self.read_angles()
        self.read_force()
        
        # 发布角度状态
        msg = Stampint32array()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = self.current_angles
        self.state_pub.publish(msg)
    
    # ==================== 清理 ====================
    
    def cleanup(self):
        """清理资源"""
        if self.ser and self.ser.is_open:
            self.get_logger().info("关闭串口...")
            self.ser.close()
        self.get_logger().info("灵巧手控制器已关闭")


# ==================== 主函数 ====================

def main(args=None):
    rclpy.init(args=args)
    node = InspireHandController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()