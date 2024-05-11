#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray  # 假设使用多浮点数数组传递关节角度信息
# 导入其他可能需要的ROS消息类型
import numpy as np
from math import pi

import sys
import os
import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
        
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

sys.path.append("..")
from scservo_sdk import *   
# 导入你的原有代码中非ROS部分的功能，例如逆运动学计算函数
from three_Inverse_kinematics import Arm


# Default setting
BAUDRATE                    = 500000        # SCServo default baudrate : 500000. 设置波特率
DEVICENAME                  = 'COM10'        # Check which port is being used on your controller. 选择串口
                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# 舵机编号，抓手为1，底盘为6
SCS_ID_1                      = 1                 # SCServo ID : 1  抓手开合
SCS_ID_2                      = 2                 # SCServo ID : 2  抓收旋转
SCS_ID_3                      = 3                 # SCServo ID : 3  第三连杆
SCS_ID_4                      = 4                 # SCServo ID : 4  第二连杆
SCS_ID_5                      = 5                 # SCServo ID : 5  第一连杆
SCS_ID_6                      = 6                 # SCServo ID : 6  控制整个机械臂旋转

SCS_MOVING_SPEED       = 1500        # SCServo moving speed 旋转速度
SCS_MOVING_ACC         = 50          # SCServo moving acc   旋转加速度


index = 0
# Initialize PortHandler instance
# Set the port path 
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = sms_sts(portHandler)


if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()


def auto_execution_routine():
   # 定义抓手的姿态
    SCS_1_GRIP_CLOSED = 2400  # 抓手闭合的位置值
    SCS_1_GRIP_OPEN = 1500    # 抓手张开的位置值
    SCS_2_ROTATE = 2047       # 抓手旋转到特定角度的位置值
    SCS_6_STATUS_VALUE=2047
    
    # 抓取目标物体的示例
    x, y = 250, 25  # 目标物体的坐标
    angle_3, angle_4, angle_5 = Arm(x, y)

    # 移动抓手到目标位置
    scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID_1, SCS_1_GRIP_OPEN, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID_2, SCS_2_ROTATE, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID_3, angle_3, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID_4, angle_4, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID_5, angle_5, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID_6, SCS_6_STATUS_VALUE, SCS_MOVING_SPEED, SCS_MOVING_ACC)


    # 等待抓手移动到指定位置
    time.sleep(2)

    # 闭合抓手
    scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID_1, SCS_1_GRIP_CLOSED, SCS_MOVING_SPEED, SCS_MOVING_ACC)

    # 等待抓手闭合
    time.sleep(1)

    #抓住物体离开箱子
    x1, y1 = x-40,y+50 
    angle_3, angle_4, angle_5 = Arm(x1, y1)
    scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID_3, angle_3, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID_4, angle_4, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID_5, angle_5, SCS_MOVING_SPEED, SCS_MOVING_ACC)

    check_and_prevent_stall(SCS_ID_1)
    # 确保所有姿态执行完毕后再退出
    print("\nAll actions executed successfully.")
    pass

def talker():
    pub_angles = rospy.Publisher('/scs_servo/angles', Float64MultiArray, queue_size=10)  # 发布关节角度
    rospy.init_node('scs_servo_controller', anonymous=True)  # 初始化节点
    rate = rospy.Rate(10)  # 10Hz的发布频率

    while not rospy.is_shutdown():
        # 根据需要调用你的功能，这里简化处理，直接发布一个示例数据
        auto_execution_routine()  # 实际应用中，你可能需要根据ROS话题或其他输入来调用
        angles_msg = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0, 0.0])  # 示例数据，替换为实际计算的关节角度
        pub_angles.publish(angles_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass