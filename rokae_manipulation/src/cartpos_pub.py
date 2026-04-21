import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import time

class CartTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('cart_test_pub')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'target_cartpos', 10)
        
        # --- 模拟参数设置 ---
        self.inference_hz = 100.0  # 推理频率 5Hz
        self.timer_period = 1.0 / self.inference_hz
        
        # 创建定时器
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.start_time = time.time()
        self.get_logger().info(f"Simulating 5Hz inference policy...")

    def timer_callback(self):
        msg = Float32MultiArray()
        
        # 获取真实经过的时间
        elapsed = time.time() - self.start_time
        
        # 基础位姿 (与机器人当前位置对应的起始点)
        base_x = 0.5635
        base_y = 0.0000
        base_z = 0.4306
        
        # 轨迹方程：在 Z 轴做正弦往复
        # 周期为 4秒 (0.25Hz), 振幅 5cm
        z_offset = 0.05 * math.sin(2 * math.pi * 0.25 * elapsed) 
        target_z = base_z + z_offset

        # 构造 4x4 齐次变换矩阵 (RowMajor)
        # 注意：这里模拟的是推理出的目标点，每 0.2s 才发布一次
        msg.data = [
            -1.0,  0.0,  0.0,  base_x,
             0.0,  1.0,  0.0,  base_y,
             0.0,  0.0, -1.0,  target_z,
             0.0,  0.0,  0.0,  1.0
        ]
        
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Publishing target z: {target_z:.4f}")

def main(args=None):
    rclpy.init(args=args)
    node = CartTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()