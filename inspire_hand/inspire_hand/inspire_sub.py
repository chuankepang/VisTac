#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Int32MultiArray
from cust_msgs.msg import Stampint32array
import numpy as np
import serial
from pynput import keyboard
from typing import List

from .ins_read import write_data_6,read_data_6,forceClb,init_pos

class InspireSub(Node):
    def __init__(self):
        super().__init__('inspire_teleop')
        self.scaled_sub = self.create_subscription(
            Int32MultiArray,
            'scaled_data',
            self.inspire_callback,
            10)
        self.pub_hand_states_ = self.create_publisher(
            Stampint32array, 'hand_states', 10)
        self.port = '/dev/ttyUSB1'
        self.baudrate = 115200
        self.joints = {
                        "Thu_MCP": 0,"Thu_IP": 1,
                        "Thu_to_Index": 2, 
                        "Ind_MCP": 3,"Ind_PIP": 4,"Ind_DIP": 5,
                        "Ind_to_Mid": 6, 
                        "Mid_MCP": 7,"Mid_PIP": 8,"Mid_DIP": 9,
                        "Mid_to_Rin": 10, 
                        "Rin_MCP": 11,"Rin_PIP": 12,"Rin_DIP": 13,
                        "Rin_to_Pin": 14, 
                        "Pin_MCP": 15,"Pin_DIP": 16,
                        "Thu_CMC": 17 
                    }
        self.force_threshold = [800,800,800,800,800,800]

        self.speed = [1000,1000,1000,1000,1000,1000]

        #手指各关节标签：拇指MCP(01),IP(02),拇指食指(03),食指MCP-PIP-DIP(04,05,06),食指中指(07),中指MCP-PIP-DIP(08,09,10),
        #               中指环指(11),环指MCP-PIP-DIP(12,13,14),环指小指(15),小指MCP-DIP(16,17),拇指CMC(18)
        self.ser = self.inspire_init()
        self.inspire_ready:bool = False
        self.disp_force:bool = False
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def inspire_callback(self, msg):
        # self.get_logger().info(f'data_receive: {msg.data}')

        write_data = self.integrate_data(msg.data)

        hand_states_msg = Stampint32array()
        hand_states_msg.header.stamp = self.get_clock().now().to_msg()
        hand_states_msg.data = write_data
        self.pub_hand_states_.publish(hand_states_msg)

        force = self.inspire_read_force(self.ser,1)
        if self.disp_force:
            self.get_logger().info(f'force: {force}')
        if self.inspire_ready:
            self.inspire_set_angle(self.ser,1,write_data)


    def openSerial(self,port,baudrate):
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baudrate
        ser.open()
        return ser
    
    def inspire_init(self):
        ser = self.openSerial(self.port,self.baudrate)
        self.get_logger().info("inspire hand serial open")
        self.get_logger().info("力校准开始，请等待设备完全停止再进行下一步操作")
        forceClb(ser,1)
        self.get_logger().info("力校准完成")
        self.inspire_set_force(ser,1,self.force_threshold)
        self.get_logger().info(f"力阈值设置为：{self.force_threshold}")
        self.inspire_set_speed(ser,1,self.speed)
        self.get_logger().info(f"速度设置为：{self.speed}")

        return ser
    
    def integrate_data(self,scaled_data)->List[int]:
        scaled_data = np.array(scaled_data)
        write_data = list(init_pos)
        # write_data = np.zeros(6)
        write_data[0] = int(self.remapping(mcp = scaled_data[self.joints["Pin_MCP"]],ip = scaled_data[self.joints["Pin_DIP"]]))
        # self.get_logger().info(f'write_data[0]: {write_data[0]}')
        write_data[1] = int(self.remapping(mcp = scaled_data[self.joints["Rin_MCP"]],ip = scaled_data[self.joints["Rin_DIP"]]))
        write_data[2] = int(self.remapping(mcp = scaled_data[self.joints["Mid_MCP"]],ip = scaled_data[self.joints["Mid_DIP"]]))
        write_data[3] = int(self.remapping(mcp = scaled_data[self.joints["Ind_MCP"]],ip = scaled_data[self.joints["Ind_DIP"]]))
        write_data[4] = int(self.remapping(mcp = scaled_data[self.joints["Thu_MCP"]],ip = scaled_data[self.joints["Thu_IP"]]))
        write_data[5] = int(self.remapping(mcp = scaled_data[self.joints["Thu_CMC"]]))
        return write_data


    def remapping(self,mcp = None,ip = None,co_thu = [0.5,0.5]):

        """
        计算传给 Inspire 手指的数据

        参数：
        - mcp: MCP 关节数据，[0-4096]
        - ip:  IP 关节数据，[0-4096]
        - co_thu: 权重系数，默认为 [0.5, 0.5]

        返回：
        - data: 计算得到的数值,范围[0-1000]
        """
        if ip is None:
            ip = mcp

        # data_range = 1000
        # 计算 ang

        ang = 1000 - (co_thu[0] * mcp  + co_thu[1] * ip)

        return ang

    def inspire_set_angle(self,ser,id,val):

        if val is not None:
            write_data_6(ser,id,'angleSet',val)

    def inspire_set_force(self,ser,id,val):
            if val is not None:
                write_data_6(ser,id,'forceSet',val)

    def inspire_set_speed(self,ser,id,val):
        if val is not None:
            write_data_6(ser,id,'speedSet',val)

    def inspire_read_angle(self,ser,id):
        return read_data_6(ser,id,'angleAct')
    
    def inspire_read_force(self,ser,id):
        force = read_data_6(ser,id,'forceAct')
        force = [(f - 65536) if f > 10000 else f for f in force]
        return force
        
    def on_press(self,key):
        try:
            if key.char == 'q':
                rclpy.shutdown()  # Stop listener
            elif key.char == '1':
                self.low_force = not self.low_force
                if self.low_force:
                    self.inspire_set_force(self.ser,1,[300,300,300,300,300,300])
                    self.get_logger().info("force theshold set to 300")
                else:
                    self.inspire_set_force(self.ser,1,self.force_threshold)
                    self.get_logger().info(f"force theshold set to {self.force_threshold}")
            elif key.char == '2':
                self.disp_force = not self.disp_force
            elif key.char == 'c':
                self.inspire_ready = not self.inspire_ready
        except AttributeError:
            pass


        
def main(args = None):
    rclpy.init(args=args) 
    node = InspireSub()  
    rclpy.spin(node) 
    rclpy.shutdown() 