import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math

class CartTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('cart_test_pub')
        # 发布到你的话题
        self.publisher_ = self.create_publisher(Float32MultiArray, 'target_cartpos', 10)
        # 以 100Hz 的频率发布 (0.01s)
        self.timer = self.create_timer(0.001, self.timer_callback)
        self.time_elapsed = 0.0

        self.get_logger().info("Starting smooth trajectory publisher (Sine wave on Z-axis)...")

    def timer_callback(self):
        msg = Float32MultiArray()
        
        # 基础平移 (基于你日志里的起点)
        base_x = 0.5635
        base_y = 0.0
        base_z = 0.4315  
        
        # 在 Z 轴上叠加一个振幅为 5cm (0.05m) 的正弦波
        # math.sin(self.time_elapsed) 频率大约为 0.16 Hz
        z_offset = 0.05 * math.sin(self.time_elapsed) 
        target_z = base_z + z_offset

        # 构造严格的 4x4 齐次变换矩阵 (行优先，16个元素)
        msg.data = [
            -1.0,  0.0,  0.0,  base_x,
             0.0,  1.0,  0.0,  base_y,
             0.0,  0.0, -1.0,  target_z,
             0.0,  0.0,  0.0,  1.0
        ]
        
        self.publisher_.publish(msg)
        self.time_elapsed += 0.01  # 每次增加 0.01 秒

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