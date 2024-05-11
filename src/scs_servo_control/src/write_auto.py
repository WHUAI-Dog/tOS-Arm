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
from scservo_sdk import *                 # Uses SCServo SDK library
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


def check_and_prevent_stall(scs_id):
    # 读取舵机当前状态下的电流和位置
    SCS_ID_I, SCS_ID_position = packetHandler.ReadPosStatus(scs_id)
    print(f"Current and position for servo {scs_id}: Current={SCS_ID_I}, Position={SCS_ID_position}")
    
    # 如果电流大于某个阈值，则调整位置以防止堵转
    while SCS_ID_I > 30:
        print(f"High current detected on servo {scs_id}, adjusting position to prevent stall...")
        # 这里我们稍微调整位置，以尝试解决堵转问题
        new_position = SCS_ID_position - 1
        scs_comm_result, scs_error = packetHandler.WritePosEx(scs_id, new_position, SCS_MOVING_SPEED, SCS_MOVING_ACC)
        if scs_comm_result != COMM_SUCCESS or scs_error != 0:
            print("Error adjusting position")
            return
        time.sleep(0.1)  # 等待舵机移动
        SCS_ID_I, SCS_ID_position = packetHandler.ReadPosStatus(scs_id)
    print("Stall prevention check complete.")

# 主程序
if __name__ == "__main__":
    try:
        auto_execution_routine()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        portHandler.closePort()