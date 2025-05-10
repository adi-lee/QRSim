# encoding=utf-8
# import itertools
import math
# import threading
import time
from socket import *
import sys
import csv

import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
import numpy as np
# from scipy.integrate import odeint
from TCP import TCPdata  # import TCP.TCPdata as TCPdata
from Control import Control

sys.path.append("..")
host = '127.0.0.1'
port = 8085
bufsiz = 2048 * 32  # 2048 * 16
addr = (host, port)

# ss = socket() # 创建服务器套接字
# ss.bind() # 把地址绑定到套接字上
# ss.listen() # 监听连接
# inf_loop: # 服务器无限循环
# cs = ss.accept() # 接受客户的连接
# comm_loop: # 通讯循环
# cs.recv()/cs.send() # 对话（接收与发送）
# cs.close() # 关闭客户套接字
# ss.close() # 关闭服务器套接字（可选）

# 创建tcp套接字，绑定，监听
tcpServerSock = socket(AF_INET, SOCK_STREAM)  # 创建TCP Socket
# AF_INET 服务器之间网络通信
# socket.SOCK_STREAM 流式socket , for TCP
tcpServerSock.bind(addr)  # 将套接字绑定到地址,
# 在AF_INET下,以元组（host,port）的形式表示地址.
tcpServerSock.listen(5)  # 操作系统可以挂起的最大连接数量，至少为1，大部分为5
fig, ax = plt.subplots()
line, = ax.plot([], [], 'o')
# 设置初始坐标轴范围
# ax.set_xlim(0, 10)  # 根据实际情况调整
# ax.set_ylim(0, 1)  # 根据实际情况调整


# def update(frame, new_x, new_y):
#     # 获取当前曲线上的数据点
#     xdata = line.get_xdata()
#     ydata = line.get_ydata()
#
#     # 将新数据点添加到曲线上
#     xdata = np.append(xdata, new_x)
#     ydata = np.append(ydata, new_y)
#
#     # 更新曲线的数据
#     line.set_data(xdata, ydata)
#     line.set_color('r')
#     fig.canvas.draw()
#     return line,

def tcp_communication():
    # global left_thrust, right_thrust, desired_speed, velocity_array, usv_2_left_thrust, usv_2_right_thrust

    # 船只的初始状态
    x_1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x y phi u v r
    x_2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x y phi u v r
    x_3 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x y phi u v r
    x_4 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x y phi u v r
    x_5 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x y phi u v r
    # 目标物的初始状态
    target = [5.0, 5.0]  # initial position of Target
    # 虚拟控制及估计速度初始状态
    # alpha_f01 = [0.0, 0.0, 0.0]  # virtual controller initial value.T
    v01_hat = [0.0, 0.0]  # ems of target velocity initial value.T
    # alpha_f02 = [0.0, 0.0, 0.0]  # virtual controller initial value.T
    v02_hat = [0.0, 0.0]  # ems of target velocity initial value.T
    # alpha_f03 = [0.0, 0.0, 0.0]  # virtual controller initial value.T
    v03_hat = [0.0, 0.0]  # ems of target velocity initial value.T
    # alpha_f04 = [0.0, 0.0, 0.0]  # virtual controller initial value.T
    v04_hat = [0.0, 0.0]  # ems of target velocity initial value.T
    # alpha_f05 = [0.0, 0.0, 0.0]  # virtual controller initial value.T
    v05_hat = [0.0, 0.0]  # ems of target velocity initial value.T
    # 控制输出， 本工程针对运动学，因此并没有用到该控制输出
    # tau_1 = [0.0, 0.0, 0.0]  # the initial value of controller.T
    # tau_2 = [0.0, 0.0, 0.0]  # the initial value of controller.T
    # tau_3 = [0.0, 0.0, 0.0]  # the initial value of controller.T
    # tau_4 = [0.0, 0.0, 0.0]  # the initial value of controller.T
    # tau_5 = [0.0, 0.0, 0.0]  # the initial value of controller.T

    # usv1_velocity = 0  # agent的实际速度
    # usv2_velocity = 0  # agent2的实际速度
    # usv3_velocity = 0  # agent3的实际速度
    # usv4_velocity = 0  # agent4的实际速度
    # usv5_velocity = 0  # agent5的实际速度
    # velocity = [usv1_velocity, usv2_velocity, usv3_velocity, usv4_velocity, usv5_velocity]

    current_time = 0  # 当前时刻
    # x0 = [x_1[0], x_1[1], x_1[2], x_1[3], x_1[4], x_1[5], x_2, x_3, target[0], target[1], alpha_f01[0],
    #       alpha_f01[1], alpha_f01[2], alpha_f02[0], alpha_f02[1], alpha_f02[2], v01_hat[0], v01_hat[1], tau_1[0],
    #       tau_1[1], tau_1[2], velocity, current_time]
    x0 = [x_1[0], x_1[1], x_1[2], x_1[3], x_1[4], x_1[5], x_2[0], x_2[1], x_2[2], x_2[3], x_2[4], x_2[5], x_3[0],
          x_3[1], x_3[2], x_3[3], x_3[4], x_3[5], x_4[0], x_4[1], x_4[2], x_4[3], x_4[4], x_4[5], x_5[0], x_5[1],
          x_5[2], x_5[3], x_5[4], x_5[5], target[0], target[1], v01_hat[0], v01_hat[1],
          v02_hat[0], v02_hat[1], v03_hat[0], v03_hat[1], v04_hat[0], v04_hat[1], v05_hat[0], v05_hat[1], current_time]
    # x = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # initial value
    # plt.ion()
    # fig, ax = plt.subplots()
    # line, = ax.plot([], [])
    # # 设置初始坐标轴范围
    # ax.set_xlim(0, 1)  # 根据实际情况调整
    # ax.set_ylim(0, 1)  # 根据实际情况调整
    # plt.show(block=False)
    i = 0  # 数据传输次数，绘图的x
    flag = 0
    xdata = []
    ydata = []
    ydata_2 = []
    ydata_3 = []
    ydata_4 = []
    ydata_5 = []
    ydata_6 = []
    ydata_7 = []
    ydata_8 = []
    ydata_9 = []
    ydata_10 = []
    ydata_11 = []
    ydata_12 = []
    ydata_13 = []
    ydata_14 = []
    ydata_15 = []
    ydata_16 = []
    ydata_17 = []
    ydata_18 = []
    ydata_19 = []
    ydata_20 = []
    ydata_21 = []
    ydata_22 = []
    ydata_23 = []
    ydata_24 = []
    ydata_25 = []
    ydata_26 = []
    ydata_27 = []
    ydata_28 = []
    ydata_29 = []
    ydata_30 = []
    ydata_31 = []
    ydata_32 = []
    ydata_33 = []
    ydata_34 = []
    ydata_35 = []
    ydata_36 = []
    ydata_37 = []
    ydata_38 = []
    ydata_39 = []
    ydata_40 = []
    ydata_41 = []
    ydata_42 = []
    ydata_43 = []
    ydata_44 = []
    ydata_45 = []
    ydata_46 = []
    ydata_47 = []
    control_core = Control.Control1()
    while True:
        print('waiting for connection')
        # udp中是recvfrom(buffersize),tcp这里用accept()；
        # tcp这里接收到的是客户端的sock对象，后面接受数据时使用socket.recv()
        tcpClientSock, addr2 = tcpServerSock.accept()  # 接受客户的连接
        # 接受TCP连接并返回（conn,address）,其中conn是新的套接字对象，
        # 可以用来接收和发送数据。
        # address是连接客户端的地址。
        print('connected from :', addr2)
        # t1 = threading.Thread(target=input_control, name='T1')  # 输入推力控制
        # t1.start()
        time_start = time.time()  # 连接成功开始计时
        while True:
            time_now = time.time()
            time_continue = time_now - time_start  # 程序运行的时长，作为x轴
            data = tcpClientSock.recv(bufsiz)  # 接收客户端发来的数据
            # print(data.decode(encoding='utf-8'))
            # if not data:
            #     break
            # # 接收数据
            ReceiveData = data.decode()
            # distance_terminal, posture, distance, warning, block, overturn, reach, velocity, destination_angle \
            #     = TCPdata.deal(ReceiveData)
            usv1_velocity_x, usv1_velocity_y, usv1_location_x, usv1_location_y, usv1_course_angle, usv1_velocity, \
                usv1_course_angle_speed, usv2_velocity_x, usv2_velocity_y, usv2_location_x, usv2_location_y,\
                usv2_course_angle, usv2_velocity, usv2_course_angle_speed, usv3_velocity_x, usv3_velocity_y, \
                usv3_location_x, usv3_location_y, usv3_course_angle, usv3_velocity, usv3_course_angle_speed, \
                target_x, target_y, usv4_velocity_x, usv4_velocity_y, usv4_location_x, usv4_location_y, \
                usv4_course_angle, usv4_velocity, usv4_course_angle_speed, usv5_velocity_x, usv5_velocity_y, \
                usv5_location_x, usv5_location_y, usv5_course_angle, usv5_velocity, usv5_course_angle_speed \
                = TCPdata.deal(ReceiveData)

            x0[0] = float(usv1_location_x) / 1000  # ue4的单位为厘米
            x0[1] = float(usv1_location_y) / 1000
            x0[2] = float(usv1_course_angle)  # 艏摇角
            x0[3] = float(usv1_velocity_x) / 1000  # 纵荡方向速度
            x0[4] = float(usv1_velocity_y) / 1000  # 横荡方向速度
            x0[5] = float(usv1_course_angle_speed)  # 艏摇角速度， 弧度
            x0[6] = float(usv2_location_x) / 1000  # ue4的单位为厘米
            x0[7] = float(usv2_location_y) / 1000
            x0[8] = float(usv2_course_angle)  # 艏摇角
            x0[9] = float(usv2_velocity_x) / 1000  # 纵荡方向速度
            x0[10] = float(usv2_velocity_y) / 1000  # 横荡方向速度
            x0[11] = float(usv2_course_angle_speed)  # 艏摇角速度， 弧度
            x0[12] = float(usv3_location_x) / 1000  # ue4的单位为厘米
            x0[13] = float(usv3_location_y) / 1000
            x0[14] = float(usv3_course_angle)  # 艏摇角
            x0[15] = float(usv3_velocity_x) / 1000  # 纵荡方向速度
            x0[16] = float(usv3_velocity_y) / 1000  # 横荡方向速度
            x0[17] = float(usv3_course_angle_speed)  # 艏摇角速度， 弧度
            x0[18] = float(usv4_location_x) / 1000  # ue4的单位为厘米
            x0[19] = float(usv4_location_y) / 1000
            x0[20] = float(usv4_course_angle)  # 艏摇角
            x0[21] = float(usv4_velocity_x) / 1000  # 纵荡方向速度
            x0[22] = float(usv4_velocity_y) / 1000  # 横荡方向速度
            x0[23] = float(usv4_course_angle_speed)  # 艏摇角速度， 弧度
            x0[24] = float(usv5_location_x) / 1000  # ue4的单位为厘米
            x0[25] = float(usv5_location_y) / 1000
            x0[26] = float(usv5_course_angle)  # 艏摇角
            x0[27] = float(usv5_velocity_x) / 1000  # 纵荡方向速度
            x0[28] = float(usv5_velocity_y) / 1000  # 横荡方向速度
            x0[29] = float(usv5_course_angle_speed)  # 艏摇角速度， 弧度
            x0[30] = float(target_x) / 1000  # target x位置
            x0[31] = float(target_y) / 1000  # target y位置
            x0[42] = float(time_continue)  # 当前时刻
            x = control_core.control(x0)
            # 速度观测器， 近似积分以更新速度观测
            x0[32] = x[3] * 0.1 + x0[32]  # usv1观测
            x0[33] = x[4] * 0.1 + x0[33]
            x0[34] = x[11] * 0.1 + x0[34]  # usv2
            x0[35] = x[12] * 0.1 + x0[35]
            x0[36] = x[19] * 0.1 + x0[36]  # usv3
            x0[37] = x[20] * 0.1 + x0[37]
            x0[38] = x[27] * 0.1 + x0[38]  # usv4
            x0[39] = x[28] * 0.1 + x0[39]
            x0[40] = x[35] * 0.1 + x0[40]  # usv5
            x0[41] = x[36] * 0.1 + x0[41]

            U1_desired = x[0]
            v1_desired = x[1]
            r1_desired = x[2]
            e_rho_1 = x[5]
            e_psi_1 = x[6]
            e_alpha12 = x[7]

            U2_desired = x[8]
            v2_desired = x[9]
            r2_desired = x[10]
            e_rho_2 = x[13]
            e_psi_2 = x[14]
            e_alpha23 = x[15]

            U3_desired = x[16]
            v3_desired = x[17]
            r3_desired = x[18]
            e_rho_3 = x[21]
            e_psi_3 = x[22]
            e_alpha34 = x[23]

            U4_desired = x[24]
            v4_desired = x[25]
            r4_desired = x[26]
            e_rho_4 = x[29]
            e_psi_4 = x[30]
            e_alpha45 = x[31]

            U5_desired = x[32]
            v5_desired = x[33]
            r5_desired = x[34]
            e_rho_5 = x[37]
            e_psi_5 = x[38]
            e_alpha51 = x[39]
            #
            # z_alpha_bar = x[40]
            # z_alpha_under = x[41]

            v1_desired = 0.08 * math.sin(0.1 * time_continue)
            v2_desired = 0.08 * math.sin(0.1 * time_continue)
            v3_desired = 0.08 * math.sin(0.1 * time_continue)
            v4_desired = 0.08 * math.sin(0.1 * time_continue)
            v5_desired = 0.08 * math.sin(0.1 * time_continue)

            # #
            u1_desired = u_real(U1_desired, v1_desired)
            u2_desired = u_real(U2_desired, v2_desired)
            u3_desired = u_real(U3_desired, v3_desired)
            u4_desired = u_real(U4_desired, v4_desired)
            u5_desired = u_real(U5_desired, v5_desired)

            r1_desired = velocity_limit_angular(r1_desired)
            r2_desired = velocity_limit_angular(r2_desired)
            r3_desired = velocity_limit_angular(r3_desired)
            r4_desired = velocity_limit_angular(r4_desired)
            r5_desired = velocity_limit_angular(r5_desired)

            u1_desired = velocity_limit(u1_desired)
            u2_desired = velocity_limit(u2_desired)
            u3_desired = velocity_limit(u3_desired)
            u4_desired = velocity_limit(u4_desired)
            u5_desired = velocity_limit(u5_desired)

            # # 测试用
            # u1_desired = 0
            # v1_desired = 0
            # r1_desired = 0
            # u2_desired = 0
            # v2_desired = 0
            # r2_desired = 0
            # u3_desired = 0
            # v3_desired = 0
            # r3_desired = 0
            # u4_desired = 0
            # v4_desired = 0
            # r4_desired = 0
            # u5_desired = 0
            # v5_desired = 0
            # r5_desired = 0

            i = i + 0.1

            xdata = np.append(xdata, time_continue)
            ydata = np.append(ydata, e_rho_1)
            ydata_2 = np.append(ydata_2, e_psi_1)
            ydata_3 = np.append(ydata_3, e_alpha34)
            ydata_4 = np.append(ydata_4, e_alpha12)
            ydata_5 = np.append(ydata_5, e_alpha23)
            ydata_6 = np.append(ydata_6, 0)
            ydata_7 = np.append(ydata_7, 0)
            ydata_8 = np.append(ydata_8, e_rho_2)
            ydata_9 = np.append(ydata_9, e_rho_3)
            ydata_10 = np.append(ydata_10, e_psi_2)
            ydata_11 = np.append(ydata_11, e_psi_3)
            ydata_12 = np.append(ydata_12, e_psi_4)
            ydata_13 = np.append(ydata_13, e_psi_5)
            ydata_14 = np.append(ydata_14, e_rho_4)
            ydata_15 = np.append(ydata_15, e_rho_5)
            ydata_16 = np.append(ydata_16, e_alpha45)
            ydata_17 = np.append(ydata_17, e_alpha51)
            ydata_18 = np.append(ydata_18, float(usv1_course_angle))
            ydata_19 = np.append(ydata_19, float(usv2_course_angle))
            ydata_20 = np.append(ydata_20, float(usv3_course_angle))
            ydata_21 = np.append(ydata_21, float(usv4_course_angle))
            ydata_22 = np.append(ydata_22, float(usv5_course_angle))
            ydata_23 = np.append(ydata_23, u1_desired)
            ydata_24 = np.append(ydata_24, u2_desired)
            ydata_25 = np.append(ydata_25, u3_desired)
            ydata_26 = np.append(ydata_26, u4_desired)
            ydata_27 = np.append(ydata_27, u5_desired)
            ydata_28 = np.append(ydata_28, v1_desired)
            ydata_29 = np.append(ydata_29, v2_desired)
            ydata_30 = np.append(ydata_30, v3_desired)
            ydata_31 = np.append(ydata_31, v4_desired)
            ydata_32 = np.append(ydata_32, v5_desired)
            ydata_33 = np.append(ydata_33, r1_desired)
            ydata_34 = np.append(ydata_34, r2_desired)
            ydata_35 = np.append(ydata_35, r3_desired)
            ydata_36 = np.append(ydata_36, r4_desired)
            ydata_37 = np.append(ydata_37, r5_desired)
            ydata_38 = np.append(ydata_38, x0[32])
            ydata_39 = np.append(ydata_39, x0[33])
            ydata_40 = np.append(ydata_40, x0[34])
            ydata_41 = np.append(ydata_41, x0[35])
            ydata_42 = np.append(ydata_42, x0[36])
            ydata_43 = np.append(ydata_43, x0[37])
            ydata_44 = np.append(ydata_44, x0[38])
            ydata_45 = np.append(ydata_45, x0[39])
            ydata_46 = np.append(ydata_46, x0[40])
            ydata_47 = np.append(ydata_47, x0[41])

            if time_continue >= 100:
                if flag < 1:

                    plt.plot(xdata, ydata)
                    plt.plot(xdata, ydata_8)
                    plt.plot(xdata, ydata_9)
                    plt.plot(xdata, ydata_14)
                    plt.plot(xdata, ydata_15)
                    plt.title("distance error")
                    # plt.ylim((-0.25, 0.5))
                    plt.show()
                    plt.plot(xdata, ydata_2)
                    plt.plot(xdata, ydata_10)
                    plt.plot(xdata, ydata_11)
                    plt.plot(xdata, ydata_12)
                    plt.plot(xdata, ydata_13)
                    plt.title("course_angle error")
                    # plt.ylim((-0.25, 0.5))
                    plt.show()
                    # plt.plot(xdata, ydata_3)
                    plt.title("agent_angle error")
                    plt.ylim((-1.0, 1.0))
                    # plt.show()
                    plt.plot(xdata, ydata_3)
                    plt.plot(xdata, ydata_4)
                    # plt.title("agent_angle12 error")
                    # plt.ylim((-0.25, 0.5))
                    plt.plot(xdata, ydata_5)
                    plt.plot(xdata, ydata_6)
                    plt.plot(xdata, ydata_7)
                    plt.plot(xdata, ydata_16)
                    # plt.plot(xdata, ydata_17)
                    plt.show()
                    # 存储数据
                    csv_file = open('simulation_data.csv', 'w', newline='', encoding='gbk')
                    writer = csv.writer(csv_file)
                    writer.writerow(xdata)
                    writer.writerow(ydata)
                    writer.writerow(ydata_2)
                    writer.writerow(ydata_3)
                    writer.writerow(ydata_4)
                    writer.writerow(ydata_5)
                    writer.writerow(ydata_6)
                    writer.writerow(ydata_7)
                    writer.writerow(ydata_8)
                    writer.writerow(ydata_9)
                    writer.writerow(ydata_10)
                    writer.writerow(ydata_11)
                    writer.writerow(ydata_12)
                    writer.writerow(ydata_13)
                    writer.writerow(ydata_14)
                    writer.writerow(ydata_15)
                    writer.writerow(ydata_16)
                    writer.writerow(ydata_17)
                    writer.writerow(ydata_18)
                    writer.writerow(ydata_19)
                    writer.writerow(ydata_20)
                    writer.writerow(ydata_21)
                    writer.writerow(ydata_22)
                    writer.writerow(ydata_23)
                    writer.writerow(ydata_24)
                    writer.writerow(ydata_25)
                    writer.writerow(ydata_26)
                    writer.writerow(ydata_27)
                    writer.writerow(ydata_28)
                    writer.writerow(ydata_29)
                    writer.writerow(ydata_30)
                    writer.writerow(ydata_31)
                    writer.writerow(ydata_32)
                    writer.writerow(ydata_33)
                    writer.writerow(ydata_34)
                    writer.writerow(ydata_35)
                    writer.writerow(ydata_36)
                    writer.writerow(ydata_37)
                    writer.writerow(ydata_38)
                    writer.writerow(ydata_39)
                    writer.writerow(ydata_40)
                    writer.writerow(ydata_41)
                    writer.writerow(ydata_42)
                    writer.writerow(ydata_43)
                    writer.writerow(ydata_44)
                    writer.writerow(ydata_45)
                    writer.writerow(ydata_46)
                    writer.writerow(ydata_47)

                    csv_file.close()
                    print("数据输出完成！")
                    flag = 1
                # 100s结束仿真，输出为0
                u1_desired = 0
                v1_desired = 0
                r1_desired = 0
                u2_desired = 0
                v2_desired = 0
                r2_desired = 0
                u3_desired = 0
                v3_desired = 0
                r3_desired = 0
                u4_desired = 0
                v4_desired = 0
                r4_desired = 0
                u5_desired = 0
                v5_desired = 0
                r5_desired = 0

            # print("实际位置:" + str(float(usv1_location_x) / 1000) + "," + str(float(usv1_location_y) / 1000))

            print("agent1输出理想速度：" + str(u1_desired) + "," + str(v1_desired) + "," + str(r1_desired))
            print("agent2输出理想速度：" + str(u2_desired) + "," + str(v2_desired) + "," + str(r2_desired))
            print("agent3输出理想速度：" + str(u3_desired) + "," + str(v3_desired) + "," + str(r3_desired))
            print("agent4输出理想速度：" + str(u4_desired) + "," + str(v4_desired) + "," + str(r4_desired))
            print("agent5输出理想速度：" + str(u5_desired) + "," + str(v5_desired) + "," + str(r5_desired))

            # msg = str('123') + ',' + str(left) + ',' + str(right) + ',' + str(0) + ',' + str(0) + ',' + \
            #       str(reach) + ',' + str(reset) + ',' + str('321')  # 测试传入UE4控制量是否正常
            msg = str('123') + ',' + str(u1_desired * 1000) + ',' + str(v1_desired * 1000) + ',' + str(
                r1_desired * 1) + ',' + str(u2_desired * 1000) + ',' + str(v2_desired * 1000) + ',' + str(
                r2_desired * 1) + ',' + str(u3_desired * 1000) + ',' + str(v3_desired * 1000) + ',' + str(
                r3_desired * 1) + ',' + str(u4_desired * 1000) + ',' + str(v4_desired * 1000) + ',' + str(
                r4_desired * 1) + ',' + str(u5_desired * 1000) + ',' + str(v5_desired * 1000) + ',' + str(
                r5_desired * 1) + ',' + str('321')
            # print(msg)

            tcpClientSock.send(msg.encode())  # 返回给客户端数据
            # time.sleep(0.1)  # 休眠0.1s
            # if tcpClientSock.send(msg.encode()):
            #     print("发送数据成功")

        tcpClientSock.close()
        tcpServerSock.close()
        return velocity_array


class TCP:
    def __init__(self):
        self.host = host
        self.port = port


def limit_thrust(thrust):  # 物理推进器组件推力限制
    if 5000 <= thrust < 10000:
        # thrust = 5000
        # thrust = 0.2 * thrust
        thrust = thrust
    elif thrust >= 10000:
        thrust = 10000
    elif thrust < 0:
        thrust = 0
    # elif -20000 < thrust <= -5000:
    #     thrust = 0.2 * thrust
    #     thrust = -5000
    # elif thrust <= -20000:
    #     thrust = -20000

    return thrust


def u_real(u, v):
    if u < v:
        u = -math.sqrt(abs(v*v-u*u))
    else:
        u = math.sqrt(abs(u*u-v*v))
    return u


def velocity_limit(velocity):  # 限幅2.5m/s
    while abs(velocity) > 5:
        velocity = 0.9*velocity
    return velocity


def velocity_limit_angular(velocity_r):  # 限幅1.5rad/s
    while abs(velocity_r) > 1.5:
        velocity_r = 0.9*velocity_r
    return velocity_r


if __name__ == '__main__':
    tcp = TCP()
    velocity_array = tcp_communication()
    plt.plot(velocity_array)
