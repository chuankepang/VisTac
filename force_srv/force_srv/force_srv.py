#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from cust_msgs.msg import Stampfloat32array
from sixd_interfaces.srv import SixDForce

import serial
import time
import struct
import numpy as np


class ForceSrv(Node):
    def __init__(self):
        super().__init__('force_srv')
        print("force_srv node started")
        self.force_srv = self.create_service(SixDForce, 'force_srv', self.force_callback)
        self.force_pub_ = self.create_publisher(Stampfloat32array, 'force_data', 10)
        self.port = '/dev/ttyUSB2'
        self.baudrate = 460800
        self.grav = 9.8015   #  gravity in Beijing
        np.set_printoptions(precision=3)
        
        self.ser = self.openSerial(self.port,self.baudrate)
        self.force_init(self.ser)
        self.isrectify = False
        self.ori_force = self.send_once(self.ser)
        self.send_continous(self.ser)
        self.timer_ = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):

        force_data = self.send_once(self.ser)
        msg = Stampfloat32array()
        if force_data is not None:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.data = force_data.tolist()
            self.force_pub_.publish(msg)
    
    def force_callback(self, request, response):
        force_data = self.send_once(self.ser)
        if force_data is not None and request.obtain_force:
            response.six_d_force = force_data.tolist()
            return response
        elif request.obtain_force is False:
            self.send_stop(self.ser)

    def openSerial(self,port,baudrate):
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baudrate
        ser.timeout = 1
        if ser.open():
            print("six_D force port opened")
        return ser
    
    def force_init(self,ser):
        bytes = [0x51,0xAA,0x0D,0x0A]
        ser.write(bytes)
        time.sleep(0.05)
        ser.read_all() # 返回帧处理掉

    def send_stop(self,ser):
        bytes = [0x51,0x00,0x0D,0x0A]
        ser.write(bytes)
        time.sleep(0.05)
        ser.read_all() # 返回帧处理掉
        pass

    def send_once(self,ser):
        self.send_stop(ser)
        bytes = [0x49,0xAA,0x0D,0x0A]
        ser.write(bytes)
        time.sleep(0.05)
        recv = ser.read_all()
        six_force = self.parsing_dataFrame(recv,b'\x49')
        return six_force

    def send_continous(self,ser):
        bytes = [0x48,0xAA,0x0D,0x0A]
        ser.write(bytes)

    def read_force(self,ser):
        recv = ser.read_all()
        six_force = self.parsing_dataFrame(recv,b'\x48')
        return six_force
    
    def parsing_dataFrame(self, dataFrame, fst_byte:bytes):
        if len(dataFrame) < 28:
            self.get_logger().warn("接收到的数据长度不足28字节")
            return
        self.get_logger().info(f"接收到数据长度：{len(dataFrame)}")
        frm_header = fst_byte + b'\xAA'
        frm_tail = b'\x0D\x0A'
        if dataFrame[0:2] != frm_header:
            self.get_logger().error(f"帧头标识错误,预期0x{frm_header.hex()}，实际值：0x{dataFrame[0:2].hex()}")
            return
        if dataFrame[-2:] != frm_tail:
            self.get_logger().error(f"帧尾错误,预期{frm_tail},实际值：{dataFrame[-2:].hex()}")
        float_bytes = dataFrame[2:26]
        try:
            Fx, Fy, Fz, Mx, My, Mz = struct.unpack('<6f', float_bytes)
            Fx, Fy, Fz, Mx, My, Mz = Fx*self.grav,Fy*self.grav,Fz*self.grav,Mx*self.grav,My*self.grav,Mz*self.grav
        except Exception as e:
            self.get_logger().error("解析浮点数时出错：{}".format(e))
            return
        if self.isrectify:
            force_ary = np.array([Fx, Fy, Fz, Mx, My, Mz])-self.ori_force
            self.get_logger().info(f'{force_ary}')
        else: # 首次上电要矫正
            force_ary = np.array([Fx, Fy, Fz, Mx, My, Mz])
            self.isrectify = True
        return force_ary



def main(args=None):
    rclpy.init(args=args)
    force_srv = ForceSrv()
    rclpy.spin(force_srv)
    force_srv.destroy_node()
    rclpy.shutdown()