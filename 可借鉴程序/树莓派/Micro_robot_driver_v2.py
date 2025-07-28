# 微型机器人上位机驱动类，包含串口与双平面机器人运动学
import pygame
import time
import serial
import math
import numpy as np
# import Robot_brain_001 as Robot_brain
# import NDI_Vega_VT_driver_v1


class Micro_Robot_base():
    J_step_buff = [0, 0, 0, 0, 0, 0]
    step_length = 0.206/1000     #机器人步长
    h = 19.72-3.4                    #机器人上下平面间距
    J_down_remainder_error=[0, 0]    #RCM运动下平面点误差补偿记录

    def __init__(self, com_port_input, baudrate_input):
        print("初始化函数")
        self.serial_port = []  # 实例化串口对象
        self.serial_port_connect(com_port_input, baudrate_input)  # 建立串口连接，参数为串口号和波特率

    # 建立串口连接
    def serial_port_connect(self, port_used, baudrate_used):
        self.serial_port = serial.Serial(
            port=port_used,  # 记得替换此处端口号
            baudrate=baudrate_used,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        # Wait a second to let the port initialize
        print(self.serial_port.read(18))
        time.sleep(1)

    # 关闭串口
    def serial_port_close(self):
        self.serial_port.close()

    # 指定电机单轴相对运动，通用函数，包含计数功能
    def on_deta_J_move(self, joint_num, deta_steps, speed):

        # 参考指令：serial_port.write("MM0-30000S0050".encode())

        if deta_steps > 0:  # 提取运动步数、方向、并转为字符串
            direct_str = "+"
            deta_steps_str = "{:0>5d}".format(deta_steps)
        else:
            direct_str = "-"
            deta_steps_str = "{:0>5d}".format(abs(deta_steps))

        speed_str = "{:0>4d}".format(speed)
        joint_num_str = str(joint_num)
        move_cmd = "MA" + joint_num_str + direct_str + deta_steps_str + "S" + speed_str

        self.serial_port.write(move_cmd.encode())  #写命令
        msg_back = self.serial_port.read(13)       #等待串口返回数据
        print("robot say: ", msg_back)

        # serial_port_read = int(self.serial_port.read(5))      此处要与下位机配合，约定上传数量
        # print("serial_port_read: ", serial_port_read)

        self.J_step_buff[joint_num] = self.J_step_buff[joint_num] + deta_steps  # 记步寄存器更新
        ##！！！ 注意缺少运动限位

        #print("move_cmd: ", move_cmd)
        print("J_step_buff: ", self.J_step_buff)
        return 1

    def on_deta_J_ALL_move(self, deta_steps_arry, speed_arr):  # 电机多轴同步运动，通用函数，包含计数功能

        for i in range(len(deta_steps_arry)):

            deta_steps = deta_steps_arry[i]
            speed = speed_arr[i]

            if deta_steps > 0:  # 提取运动步数、方向、并转为字符串
                direct_str = "+"
                deta_steps_str = "{:0>5d}".format(deta_steps)
            else:
                direct_str = "-"
                deta_steps_str = "{:0>5d}".format(abs(deta_steps))

            speed_str = "{:0>4d}".format(speed)
            joint_num_str = str(i)
            move_cmd = "MM" + joint_num_str + direct_str + deta_steps_str + "S" + speed_str

            self.serial_port.write(move_cmd.encode())
            #print("move_cmd: ", move_cmd)

            self.J_step_buff[i] = self.J_step_buff[i] + deta_steps  # 记步寄存器更新
            ##！！！ 注意缺少运动限位

        msg_back = self.serial_port.read(13)       #等待串口返回数据
        print("robot say: ", msg_back)
        print("on_deta_J_ALL_move ok! ")
        print("J_step_buff: ", self.J_step_buff)

    def on_J_move(self, joint_num, Target_step_point, speed):  # 指定电机单轴运动到绝对步数，通用函数，包含计数功能
        deta_steps = Target_step_point - self.J_step_buff[joint_num]   # 计算相对步数
        self.on_deta_J_move(joint_num, deta_steps, speed)   #运动到目标步数
        print("on_J_move axis ok!")

    def on_J_ALL_move(self, Target_step_point_array, speed_array):  # 多轴运动到绝对步数，通用函数，包含计数功能
        deta_steps_array = np.array(Target_step_point_array) - np.array(self.J_step_buff)  # 计算相对步数
        self.on_deta_J_ALL_move(deta_steps_array, speed_array)
        print("on_J_ALL_move ok! ")

    def MOVE_Home(self, speed):  # 机器人运动到Home位置，所有关节角清零
        Target_step_point_array_1 = [-20000, -20000, -20000, -20000, +20000, -20000]
        Target_step_point_array_2 = [23000, 18000, 20000, 20000, -20000, 20000]
        speed_array = [speed, speed, speed, speed, speed, speed]

        self.on_deta_J_ALL_move(Target_step_point_array_1, speed_array)
        self.on_deta_J_ALL_move(Target_step_point_array_1, speed_array)
        self.on_deta_J_ALL_move(Target_step_point_array_2, speed_array)
        self.J_step_buff = [0, 0, 0, 0, 0, 0]
        print("J_step_buff: ", self.J_step_buff)
        print('Move Home ok')

    def MOVE_P(self, Robot_P_Position, speed):  # 机器人运动到指定位姿，xyz+欧拉角

        J = [0, 0, 0, 0, 0, 0]    # J变量初始化
        x_up = Robot_P_Position[0]
        y_up = Robot_P_Position[1]
        z_up = Robot_P_Position[2]
        pusai = Robot_P_Position[3]
        theta = Robot_P_Position[4]
        fai = Robot_P_Position[5]

        J[0] = y_up / self.step_length
        J[1] = x_up / self.step_length

        J[2] = (y_up - math.tan(-pusai) * self.h) / self.step_length
        J[3] = (x_up - math.tan(theta) * self.h) / self.step_length

        J = np.trunc(J)
        J = J.astype(np.int32)  #转为整数
        speed_array = [speed, speed, speed, speed, 0, 0]
        self.on_J_ALL_move(J, speed_array)  #机器人运动到目标位置，以平均速度
        print('Move_P ok')
        return 1

    def MOVE_RCM_deta_J_up(self,J_up_deta,Point_rcm,speed):
        # 居家编程，未经测试
        # 根据上平面运动增量，控制机器人做RCM运动,要求上下平面点均处于RCM运动约束上
        x_rcm = Point_rcm[0] #rcm点坐标
        y_rcm = Point_rcm[1]
        z_rcm = Point_rcm[2]

        JX_up_deta = J_up_deta[1] # 机器人上平面中心点X轴关节增量
        JY_up_deta = J_up_deta[0] # 机器人上平面中心点Y轴关节增量

        J_ini=self.J_step_buff      #读取当前的关键变量值作为初始值

        JX_down_remainder_error = self.J_down_remainder_error[1] # 下平面X轴余数误差存储，误差超过步长时则进一步或推一步步长
        JY_down_remainder_error = self.J_down_remainder_error[0] # 下平面Y轴余数误差存储，误差超过步长时则进一步或推一步步长

        z_up_ini = 0 * self.step_length # 上平面中心点z轴坐标，默认为0
        x_up_ini = J_ini[1] * self.step_length # 上平面中心点x轴坐标
        y_up_ini = J_ini[0] * self.step_length # 上平面中心点y轴坐标(d)

        z_down_ini = -self.h #下平面中心点z轴初始坐标，默认为 - h
        x_down_ini = J_ini[3] * self.step_length # 下平面中心点x轴初始坐标
        y_down_ini = J_ini[2] * self.step_length # 下平面中心点y轴初始坐标

        y_up_deta = JY_up_deta * self.step_length # 上平面中心点y轴坐标增量
        x_up_deta = JX_up_deta * self.step_length # 上平面中心点x轴坐标增量

        z_up = 0 * self.step_length      # 上平面中心点z轴坐标，默认为0
        y_up = y_up_ini + y_up_deta      # 上平面中心点y轴坐标
        x_up = x_up_ini + x_up_deta      # 上平面中心点y轴坐标

        n_ratio = (z_up - z_rcm) / ((z_up - z_rcm) - self.h) # 远心点相似三角形直角边之比

        # 根据求导公式获得下平面坐标点变化
        z_down_deta = 0
        y_down_deta = (1 / n_ratio) * y_up_deta
        x_down_deta = (1 / n_ratio) * x_up_deta

        JX_down_deta = round(x_down_deta / self.step_length)
        JX_dwon_remainder_error = JX_down_remainder_error + (
                    round(x_down_deta / self.step_length) * self.step_length - x_down_deta)

        if (JX_down_remainder_error >= self.step_length):
            JX_down_deta = JX_down_deta + 1 # 补偿除整余数误差
            JX_down_remainder_error = JX_down_remainder_error - self.step_length # 补偿后误差计数更新

        elif (JX_down_remainder_error <= -self.step_length):
            JX_down_deta = JX_down_deta - 1 # 补偿除整余数误差
            JX_down_remainder_error = JX_down_remainder_error + self.step_length # 补偿后误差计数更新


        JY_down_deta = round(y_down_deta / self.step_length)
        JY_ddown_remainder_error = JY_down_remainder_error + (round(y_down_deta / self.step_length) * self.step_length - y_down_deta);

        if (JY_down_remainder_error >= self.step_length):
            JY_down_deta = JY_down_deta + 1 # 补偿除整余数误差
            JY_dwon_remainder_error = JY_down_remainder_error - self.step_length # 补偿后误差计数更新

        elif(JY_down_remainder_error <= -self.step_length):
            JY_down_deta = JY_down_deta - 1 # 补偿除整余数误差
            JY_dwon_remainder_error = JY_down_remainder_error + self.step_length # 补偿后误差计数更新

        self.J_dwon_remainder_error = [JY_down_remainder_error, JX_down_remainder_error]
        J_deta = [JY_up_deta, JX_up_deta, JY_down_deta, JX_down_deta,  0,  0]                       # 输出符合RCM约束的增量关节角度

        speed_array = [speed, speed, speed, speed, 0, 0]
        self.on_deta_J_ALL_move(J_deta, speed_array)
        print('MOVE_RCM_deta_J_up ok')
        return J_deta

    def Get_Current_P_position(self):
        # 居家编程，未经测试
        # 根据关节位置计算末端法兰向量
        J = self.J_step_buff
        x_up = self.step_length * J[1] # 坐标原点，上平面中心点
        y_up = self.step_length * J[0]
        x_down = self.step_length * J[3] # 下平面中心点
        y_down = self.step_length * J[2]

        # 运动点坐标
        P_up = np.array([x_up, y_up, 0])   # 上平面坐标P0
        # P_down = np.array([x_down, y_down, -self.h])   # 下平面坐标P1
        # Vecor = (P_up - P_down) / np.linalg.norm(P_up-P_down)   # 定义轴线方向为下平面运动点指向上平面运动点
        # return_data = np.array([P_up, Vecor])

        pusai = -math.atan((y_up - y_down) / self.h)   # 绕x轴旋转
        theta = math.atan((x_up - x_down) / self.h)   # 绕y轴旋转
        fai = 0  # 绕z轴旋转

        Current_Robot_P_Position = [P_up[0], P_up[1], P_up[2], pusai, theta, fai]

        #计算当前机器人位置矩阵
        #Current_Robot_T_Position = Robot_brain.XYZABC2MAT(Current_Robot_P_Position, format="none degree")

        print("Get_Current_P_position ok!")
        return Current_Robot_P_Position


    def Get_Current_J_position(self):
        # 居家编程，未经测试
        # 读取末端关节位置
        Current_Robot_J_Position = self.J_step_buff

        print("Get_Current_J_position ok")
        return Current_Robot_J_Position.copy()

    # 指定电机单轴相对运动，通用函数，包含计数功能
    def on_drill_run(self, dri, speed):
        speed_num = speed
        speed_num_str = "{:03}".format(speed_num)
        if dri==-1:
            DR_motor_cmd_str = "DR0-00000S0" + speed_num_str
        elif dri==1:
            DR_motor_cmd_str = "DR0+00000S0" + speed_num_str

        self.serial_port.write(DR_motor_cmd_str.encode())

# # 机器人实例化
# Micro_Robot_Used = Micro_Robot_base(com_port_input="COM5", baudrate_input=9600)
# MoveHomeSpeed = 50
# MoveSpeed_used = 100

## ndi实例化
#NDI_Vega_used=NDI_Vega_VT_driver_v1.NDI_Vega_VT()
#

# deta_steps_arry_used = [5000, 5000, 5000, 5000, 1000, 1000]
# deta_steps_arry_used1 = [-5000, -5000, -5000, -5000, 1000, 1000]
# speed_arry_used = [200, 200, 200, 200, 200, 200]
# Robot_P_Position = [0, 3, 0, 0*math.pi/180, 00*math.pi/180, 0*math.pi/180]
# Robot_P_Position_home = [0, 0, 0, 0*math.pi/180, 0*math.pi/180, 0*math.pi/180]

# Micro_Robot_Used.MOVE_Home(MoveHomeSpeed)   # 返回Home位置

## 测试单轴绝对运动
# Micro_Robot_Used.on_J_move(3, 20000, 150)
# Micro_Robot_Used.on_J_move(3, 0, 150)
# Micro_Robot_Used.on_J_move(3, -20000, 150)
# Micro_Robot_Used.on_J_move(3, 0, 150)

## 测试多轴绝对运动
# Micro_Robot_Used.on_J_ALL_move([10000,10000,10000,10000,1000,1000], speed_arry_used)
# Micro_Robot_Used.on_J_ALL_move([-10000,-10000,-10000,-10000,-1000,1000],speed_arry_used)
# Micro_Robot_Used.on_J_ALL_move([-10000,+10000,-10000,+10000,-1000,1000],speed_arry_used)
# Micro_Robot_Used.on_J_ALL_move([0,0,0,0,0,0],speed_arry_used)

# 测试RCM增量运动
# Micro_Robot_Used.MOVE_Home(MoveHomeSpeed)   # 返回Home位置
# Point_rcm = [0, 0, -50]     #设置RCM点
# J_up_deta = [15000, 0]
#
# Micro_Robot_Used.MOVE_RCM_deta_J_up([15000, 0], Point_rcm, MoveSpeed_used)
# for i in range(5):
#     Micro_Robot_Used.MOVE_RCM_deta_J_up([-30000, 0], Point_rcm, MoveSpeed_used)
#     Micro_Robot_Used.MOVE_RCM_deta_J_up([30000, 0], Point_rcm, MoveSpeed_used)
# #
# for i in range(1):
#     Micro_Robot_Used.MOVE_RCM_deta_J_up(J_up_deta, Point_rcm, MoveSpeed_used)
# for j in range(2):
#     J_up_deta = [0, 15000]
#     for i in range(1):
#         Micro_Robot_Used.MOVE_RCM_deta_J_up(J_up_deta, Point_rcm, MoveSpeed_used)
#     J_up_deta = [-30000, 0]
#     for i in range(1):
#         Micro_Robot_Used.MOVE_RCM_deta_J_up(J_up_deta, Point_rcm, MoveSpeed_used)
#     J_up_deta = [0, -30000]
#     for i in range(1):
#         Micro_Robot_Used.MOVE_RCM_deta_J_up(J_up_deta, Point_rcm, MoveSpeed_used)
#     J_up_deta = [30000, 0]
#     for i in range(1):
#         Micro_Robot_Used.MOVE_RCM_deta_J_up(J_up_deta, Point_rcm, MoveSpeed_used)
#     J_up_deta = [0, 15000]
#     for i in range(1):
#         Micro_Robot_Used.MOVE_RCM_deta_J_up(J_up_deta, Point_rcm, MoveSpeed_used)


#Micro_Robot_Used.MOVE_P(Robot_P_Position_home, MoveSpeed_used)   # 回到Home位置
# Micro_Robot_Used.MOVE_P(Robot_P_Position, MoveSpeed_used)   # 运动到指定位置P变量
# print(Micro_Robot_Used.Get_Current_P_position())            # 打印当前位置P变量
#
# Micro_Robot_Used.MOVE_P(Robot_P_Position_home, MoveSpeed_used)   # 运动到指定位置P变量
# print(Micro_Robot_Used.Get_Current_P_position())            # 打印当前位置P变量

# Micro_Robot_Used.on_deta_J_ALL_move(deta_steps_arry=deta_steps_arry_used, speed_arr=speed_arry_used)
# time.sleep(3)
# Micro_Robot_Used.on_deta_J_ALL_move(deta_steps_arry=deta_steps_arry_used1, speed_arr=speed_arry_used)
#

#Micro_Robot_Used.on_deta_J_move(joint_num=0, deta_steps=1000, speed=200)
# Micro_Robot_Used.on_deta_J_move(joint_num=0, deta_steps=-1000, speed=200)

#Micro_Robot_Used.serial_port_close()
