# @Time: 20250111
# @Author: adi
# for chap02 3D simulation
import math
# import time

import numpy as np


def limit_thrust(thrust):  # 物理推进器组件推力限制
    if thrust >= 10000:
        # thrust = 10000
        thrust = 0.1 * thrust
    elif thrust <= -10000:
        thrust = 0.1 * thrust
        # thrust = -10000

    return thrust


class Control1:
    def __init__(self):
        self.r_des = 5  # 围捕半径  # ue4的单位为厘米
        self.agent_radius = 0.5  # agent的几何半径
        self.target_radius = 1  # target的几何半径
        self.rho_min = self.agent_radius + self.target_radius  # 避免碰撞的最小距离
        self.rho_max = 30  # 围捕的最大感知距离
        self.w_bar = 0.3  # 环绕速度  # 对应0.3m/s
        self.k_rho1 = 2  # 控制参数
        self.k_rho2 = 2  # 控制参数
        self.k_rho3 = 2  # 控制参数
        self.k_rho4 = 2  # 控制参数
        self.k_rho5 = 2  # 控制参数
        self.k_r1 = 2  # 控制参数
        self.k_r2 = 2  # 控制参数
        self.k_r3 = 2  # 控制参数
        self.k_r4 = 2  # 控制参数
        self.k_r5 = 2  # 控制参数
        self.k_alpha_1 = 2.5  # 控制参数
        self.k_alpha_2 = 2.5  # 控制参数
        self.k_alpha_3 = 2.5  # 控制参数
        self.k_alpha_4 = 2.5  # 控制参数
        self.k_alpha_5 = 2.5

        self.k_beta1 = 1
        self.k_beta2 = 1
        self.k_beta3 = 1
        self.k_beta4 = 1
        self.k_beta5 = 1

        self.hat_dot_beta_angle_1 = 0
        self.hat_dot_beta_angle_2 = 0
        self.hat_dot_beta_angle_3 = 0
        self.hat_dot_beta_angle_4 = 0
        self.hat_dot_beta_angle_5 = 0

        self.Gamma = np.diag([0.1] * 2)  # 参数更新率，生成对角为0.1的三维对角矩阵
        self.K_0 = np.diag([0.3] * 2)  # 控制参数
        # self.mu = np.diag([0.1] * 3)  # 控制参数
        # self.K1_z = np.diag([500] * 3)  # 控制参数
        self.alpha_min = math.pi / 6  # 智能体之间的最小夹角
        self.alpha_max = (2 * math.pi - self.alpha_min/4)  # 智能体之间的最大夹角
        self.alpha_des12 = 2 * math.pi / 5  # 智能体12之间理想的夹角
        self.alpha_des23 = 2 * math.pi / 5  # 智能体23之间理想的夹角
        self.alpha_des34 = 2 * math.pi / 5  # 智能体34之间理想的夹角
        self.alpha_des45 = 2 * math.pi / 5  # 智能体45之间理想的夹角
        self.e_underline_rho = self.r_des - self.rho_min  # 障碍函数的下限
        self.e_bar_rho = self.rho_max - self.r_des  # 障碍函数的上限
        self.beta_alpha_0 = 1  # barrier function bound angle error
        self.beta_alpha_inf = 0.06
        self.beta_gamma = 0.1
        self.vehicle_d = 0.1

        # debug
        self.times = 0
    def control(self, x):
        # 当前时刻
        current_time = x[42]
        # target的状态
        target_location_x = x[30]
        target_location_y = x[31]
        v_target = np.array([[0], [0.2]])

        # agent5的状态
        usv5_location_x = x[24]  # agent的x位置
        usv5_location_y = x[25]  # agent的y位置
        usv5_course_angle = x[26] * math.pi / 180  # agent的朝向 转换成为弧度
        u_5 = x[27]
        v_5 = x[28]
        r_5 = x[29]

        v05_hat = np.array([[x[40]], [x[41]]])
        beta_angle_5 = math.atan2(v_5, u_5)
        #############################################
        # slide angle deal
        if abs(u_5) < 0.05 and abs(v_5) < 0.05:
            beta_angle_5 = 0
        #############################################
        usv_yaw_angle_plus_beta_5 = beta_angle_5 + usv5_course_angle
        
        H_5 = np.array([[math.cos(usv_yaw_angle_plus_beta_5), -self.vehicle_d * math.sin(usv5_course_angle)],
                        [math.sin(usv_yaw_angle_plus_beta_5),
                         self.vehicle_d * math.cos(usv5_course_angle)]])  # 旋转矩阵
        rho_5 = math.sqrt((math.pow(
            usv5_location_x + self.vehicle_d * math.cos(usv_yaw_angle_plus_beta_5) - target_location_x, 2) + math.pow(
            usv5_location_y + self.vehicle_d * math.sin(usv_yaw_angle_plus_beta_5) - target_location_y, 2)))  # 计算两船距离
        e_rho_5 = rho_5 - self.r_des  # 围捕的距离误差
        print("agent5距离误差：" + str(e_rho_5))
        p_5 = np.array([[usv5_location_x], [usv5_location_y]]) \
              + self.vehicle_d * np.array(
            [[math.cos(usv_yaw_angle_plus_beta_5)], [math.sin(usv_yaw_angle_plus_beta_5)]])
        p_0 = np.array([[target_location_x], [target_location_y]])  # Target的位置向量， [x_0, y_0]^T
        psi_5 = (p_5 - p_0) / rho_5  # Target与agent之间的连线的单位向量
        phi_5 = usv5_course_angle  # 方便移植，不然老是忘记这个是朝向
        theta_5 = math.atan2(psi_5[1], psi_5[0])  # Target与agent之间连线的方位角
        bar_psi_5 = np.array(
            [[math.cos(theta_5 + math.pi / 2)],
             [math.sin(theta_5 + math.pi / 2)]])  # 与psi角垂直的向量，-pi/2表示逆时针旋转,与MATLAB坐标系不同

        eta_rho_5 = Control1.barrier(self, e_rho_5)  # 障碍函数
        d_eta_rho_5 = Control1.d_barrier(self, e_rho_5)  # 障碍函数的导数

        # 虚拟控制律
        k_5 = np.array([[self.k_rho5], [self.k_r5]])
        c_5 = np.dot(np.linalg.inv(H_5), (
                (-k_5 * (
                        eta_rho_5 * d_eta_rho_5 * psi_5)) + bar_psi_5 * self.w_bar
                * rho_5 + v05_hat))
        # 速度观测器
        S_5 = -eta_rho_5 * d_eta_rho_5 * psi_5
        temp2 = (- S_5.T - np.dot(self.K_0, v05_hat))  # 临时变量
        d_v05_hat = np.dot(self.Gamma, temp2)

        # 侧滑角观测器
        d_hat_dot_beta_angle_5 = -self.k_beta5 * self.hat_dot_beta_angle_5
        self.hat_dot_beta_angle_5 = 0.1 * d_hat_dot_beta_angle_5 + self.hat_dot_beta_angle_5
        # self.hat_dot_beta_angle = 0

        alpha_usv_U5 = c_5[0][0]
        alpha_usv_w5 = c_5[1][0]
        # 限幅
        # alpha_usv_U5 = speed__limit_proportion(alpha_usv_U5, 0)
        # alpha_usv_w5 = speed__limit_proportion(alpha_usv_w5, 1)

        e_phi_5 = (phi_5 - (math.pi / 2 + theta_5))  # 朝向角误差
        while abs(e_phi_5) >= math.pi:
            if e_phi_5 > 0:
                e_phi_5 = e_phi_5 - 2 * math.pi
            else:
                e_phi_5 = e_phi_5 + 2 * math.pi

        self.times = self.times + 1
        if self.times == 2:
            self.times = 100



        # agent4的状态
        usv4_location_x = x[18]  # agent的x位置
        usv4_location_y = x[19]  # agent的y位置
        usv4_course_angle = x[20] * math.pi / 180  # agent的朝向 转换成为弧度
        u_4 = x[21]
        v_4 = x[22]
        r_4 = x[23]
        v04_hat = np.array([[x[38]], [x[39]]])
        beta_angle_4 = math.atan2(v_4, u_4)
        #############################################
        # slide angle deal
        if abs(u_4) < 0.05 and abs(v_4) < 0.05:
            beta_angle_4 = 0
        #############################################
        usv_yaw_angle_plus_beta_4 = beta_angle_4 + usv4_course_angle

        H_4 = np.array([[math.cos(usv_yaw_angle_plus_beta_4), -self.vehicle_d * math.sin(usv4_course_angle)],
                        [math.sin(usv_yaw_angle_plus_beta_4),
                         self.vehicle_d * math.cos(usv4_course_angle)]])  # 旋转矩阵
        rho_4 = math.sqrt((math.pow(usv4_location_x - target_location_x, 2) + math.pow(usv4_location_y -
                                                                                       target_location_y, 2)))  # 计算两船距离
        e_rho_4 = rho_4 - self.r_des  # 围捕的距离误差
        print("agent4距离误差：" + str(e_rho_4))
        eta_rho_4 = Control1.barrier(self, e_rho_4)  # 障碍函数
        d_eta_rho_4 = Control1.d_barrier(self, e_rho_4)  # 障碍函数的导数
        p_4 = np.array([[usv4_location_x], [usv4_location_y]])  # 围捕船的位置向量，[x, y]^T
        p_0 = np.array([[target_location_x], [target_location_y]])  # Target的位置向量， [x_0, y_0]^T
        psi_4 = (p_4 - p_0) / rho_4  # Target与agent之间的连线的单位向量
        phi_4 = usv4_course_angle  # 方便移植，不然老是忘记这个是朝向
        theta_4 = math.atan2(psi_4[1], psi_4[0])  # Target与agent之间连线的方位角
        bar_psi_4 = np.array(
            [[math.cos(theta_4 + math.pi / 2)],
             [math.sin(theta_4 + math.pi / 2)]])  # 与psi角垂直的向量，-pi/2表示逆时针旋转,与MATLAB坐标系不同
        p_04 = p_4 - p_0
        p_05 = p_5 - p_0
        angle_alpha_45 = math.acos(np.dot(p_04.T, p_05) / (np.linalg.norm(p_04) * np.linalg.norm(p_05)))
        e_alpha_45 = angle_alpha_45 - self.alpha_des45
        beta_alpha45 = Control1.beta(self, current_time)
        d_beta_alpha45 = Control1.d_beta(self, current_time)
        z_alpha45 = e_alpha_45 / beta_alpha45
        z_alpha45_under = self.alpha_des45 - self.alpha_min
        z_alpha45_bar = self.alpha_max - self.alpha_des45
        eta_alpha45 = Control1.z_barrier(self, z_alpha45, z_alpha45_bar, z_alpha45_under)
        d_eta_alpha45 = Control1.d_z_barrier(self, z_alpha45, z_alpha45_bar, z_alpha45_under)

        # 虚拟控制律
        c_4 = np.dot(np.linalg.inv(H_4), (-(self.k_rho4 * eta_rho_4 * d_eta_rho_4 * psi_4) +
                                                    bar_psi_4 * (-self.k_alpha_4 *
                                                                 (eta_alpha45 * d_eta_alpha45 * (
                                                                         beta_alpha45 * rho_4)) +
                                                                 e_alpha_45 * d_beta_alpha45 * rho_4 / beta_alpha45) + v04_hat))
        S_4 = eta_rho_4 * d_eta_rho_4*psi_4.T + 1/rho_4*(eta_alpha45*d_eta_alpha45)*bar_psi_4.T
        d_v04_hat = np.dot(self.Gamma, (-S_4.T - np.dot(self.K_0, v04_hat)))

        # 侧滑角观测器
        d_hat_dot_beta_angle_4 = -self.k_beta4 * self.hat_dot_beta_angle_4
        self.hat_dot_beta_angle_4 = 0.01 * d_hat_dot_beta_angle_4 + self.hat_dot_beta_angle_4
        # self.hat_dot_beta_angle = 0

        alpha_usv_U4 = c_4[0][0]
        alpha_usv_w4 = c_4[1][0]
        # 限幅
        # alpha_usv_U4 = speed__limit_proportion(alpha_usv_U4, 0)
        # alpha_usv_w4 = speed__limit_proportion(alpha_usv_w4, 1)

        e_phi_4 = (phi_4 - (math.pi / 2 + theta_4))  # 朝向角误差
        while abs(e_phi_4) >= math.pi:
            if e_phi_4 > 0:
                e_phi_4 = e_phi_4 - 2 * math.pi
            else:
                e_phi_4 = e_phi_4 + 2 * math.pi

        # agent3的状态
        usv3_location_x = x[12]  # agent的x位置
        usv3_location_y = x[13]  # agent的y位置
        usv3_course_angle = x[14] * math.pi / 180  # agent的朝向 转换成为弧度
        u_3 = x[15]
        v_3 = x[16]
        r_3 = x[17]
        v03_hat = np.array([[x[36]], [x[37]]])
        beta_angle_3 = math.atan2(v_3, u_3)
        #############################################
        # slide angle deal
        if abs(u_3) < 0.05 and abs(v_3) < 0.05:
            beta_angle_3 = 0
        #############################################
        usv_yaw_angle_plus_beta_3 = beta_angle_3 + usv3_course_angle

        H_3 = np.array([[math.cos(usv_yaw_angle_plus_beta_3), -self.vehicle_d * math.sin(usv3_course_angle)],
                        [math.sin(usv_yaw_angle_plus_beta_3),
                         self.vehicle_d * math.cos(usv3_course_angle)]])  # 旋转矩阵
        rho_3 = math.sqrt((math.pow(usv3_location_x - target_location_x, 2) + math.pow(usv3_location_y -
                                                                                       target_location_y, 2)))  # 计算两船距离
        e_rho_3 = rho_3 - self.r_des  # 围捕的距离误差
        print("agent3距离误差：" + str(e_rho_3))
        eta_rho_3 = Control1.barrier(self, e_rho_3)  # 障碍函数
        d_eta_rho_3 = Control1.d_barrier(self, e_rho_3)  # 障碍函数的导数
        p_3 = np.array([[usv3_location_x], [usv3_location_y]])  # 围捕船的位置向量，[x, y]^T
        p_0 = np.array([[target_location_x], [target_location_y]])  # Target的位置向量， [x_0, y_0]^T
        psi_3 = (p_3 - p_0) / rho_3  # Target与agent之间的连线的单位向量
        phi_3 = usv3_course_angle  # 方便移植，不然老是忘记这个是朝向
        theta_3 = math.atan2(psi_3[1], psi_3[0])  # Target与agent之间连线的方位角
        bar_psi_3 = np.array(
            [[math.cos(theta_3 + math.pi / 2)],
             [math.sin(theta_3 + math.pi / 2)]])  # 与psi角垂直的向量，-pi/2表示逆时针旋转,与MATLAB坐标系不同
        p_03 = p_3 - p_0
        p_04 = p_4 - p_0
        angle_alpha_34 = math.acos(np.dot(p_03.T, p_04) / (np.linalg.norm(p_03) * np.linalg.norm(p_04)))
        e_alpha_34 = angle_alpha_34 - self.alpha_des34
        beta_alpha34 = Control1.beta(self, current_time)
        d_beta_alpha34 = Control1.d_beta(self, current_time)
        z_alpha34 = e_alpha_34 / beta_alpha34
        z_alpha34_under = self.alpha_des34 - self.alpha_min
        z_alpha34_bar = self.alpha_max - self.alpha_des34
        eta_alpha34 = Control1.z_barrier(self, z_alpha34, z_alpha34_bar, z_alpha34_under)
        d_eta_alpha34 = Control1.d_z_barrier(self, z_alpha34, z_alpha34_bar, z_alpha34_under)

        # 虚拟控制律
        c_3 = np.dot(np.linalg.inv(H_3), (-(self.k_rho3 * eta_rho_3 * d_eta_rho_3 * psi_3) +
                                          bar_psi_3 * (-self.k_alpha_3 *
                                                       (eta_alpha34 * d_eta_alpha34 * (
                                                               beta_alpha34 * rho_3)) +
                                                       e_alpha_34 * d_beta_alpha34 * rho_3 / beta_alpha34) + v03_hat))
        S_3 = eta_rho_3 * d_eta_rho_3 * psi_3.T + 1 / rho_3 * (eta_alpha34 * d_eta_alpha34) * bar_psi_3.T
        d_v03_hat = np.dot(self.Gamma, (-S_3.T - np.dot(self.K_0, v03_hat)))

        # 侧滑角观测器
        d_hat_dot_beta_angle_3 = -self.k_beta3 * self.hat_dot_beta_angle_3
        self.hat_dot_beta_angle_3 = 0.01 * d_hat_dot_beta_angle_3 + self.hat_dot_beta_angle_3
        # self.hat_dot_beta_angle = 0

        alpha_usv_U3 = c_3[0][0]
        alpha_usv_w3 = c_3[1][0]
        # 限幅
        # alpha_usv_U3 = speed__limit_proportion(alpha_usv_U3, 0)
        # alpha_usv_w3 = speed__limit_proportion(alpha_usv_w3, 1)

        e_phi_3 = (phi_3 - (math.pi / 2 + theta_3))  # 朝向角误差
        while abs(e_phi_3) >= math.pi:
            if e_phi_3 > 0:
                e_phi_3 = e_phi_3 - 2 * math.pi
            else:
                e_phi_3 = e_phi_3 + 2 * math.pi

        # agent2的状态
        usv2_location_x = x[6]  # agent的x位置
        usv2_location_y = x[7]  # agent的y位置
        usv2_course_angle = x[8] * math.pi / 180  # agent的朝向 转换成为弧度
        u_2 = x[9]
        v_2 = x[10]
        r_2 = x[11]
        v02_hat = np.array([[x[33]], [x[35]]])
        beta_angle_2 = math.atan2(v_2, u_2)
        #############################################
        # slide angle deal
        if abs(u_2) < 0.05 and abs(v_2) < 0.05:
            beta_angle_2 = 0
        #############################################
        usv_yaw_angle_plus_beta_2 = beta_angle_2 + usv2_course_angle

        H_2 = np.array([[math.cos(usv_yaw_angle_plus_beta_2), -self.vehicle_d * math.sin(usv2_course_angle)],
                        [math.sin(usv_yaw_angle_plus_beta_2),
                         self.vehicle_d * math.cos(usv2_course_angle)]])  # 旋转矩阵
        rho_2 = math.sqrt((math.pow(usv2_location_x - target_location_x, 2) + math.pow(usv2_location_y -
                                                                                       target_location_y, 2)))  # 计算两船距离
        e_rho_2 = rho_2 - self.r_des  # 围捕的距离误差
        print("agent2距离误差：" + str(e_rho_2))
        eta_rho_2 = Control1.barrier(self, e_rho_2)  # 障碍函数
        d_eta_rho_2 = Control1.d_barrier(self, e_rho_2)  # 障碍函数的导数
        p_2 = np.array([[usv2_location_x], [usv2_location_y]])  # 围捕船的位置向量，[x, y]^T
        p_0 = np.array([[target_location_x], [target_location_y]])  # Target的位置向量， [x_0, y_0]^T
        psi_2 = (p_2 - p_0) / rho_2  # Target与agent之间的连线的单位向量
        phi_2 = usv2_course_angle  # 方便移植，不然老是忘记这个是朝向
        theta_2 = math.atan2(psi_2[1], psi_2[0])  # Target与agent之间连线的方位角
        bar_psi_2 = np.array(
            [[math.cos(theta_2 + math.pi / 2)],
             [math.sin(theta_2 + math.pi / 2)]])  # 与psi角垂直的向量，-pi/2表示逆时针旋转,与MATLAB坐标系不同
        p_02 = p_2 - p_0
        p_03 = p_3 - p_0
        angle_alpha_23 = math.acos(np.dot(p_02.T, p_03) / (np.linalg.norm(p_02) * np.linalg.norm(p_03)))
        e_alpha_23 = angle_alpha_23 - self.alpha_des23
        beta_alpha23 = Control1.beta(self, current_time)
        d_beta_alpha23 = Control1.d_beta(self, current_time)
        z_alpha23 = e_alpha_23 / beta_alpha23
        z_alpha23_under = self.alpha_des23 - self.alpha_min
        z_alpha23_bar = self.alpha_max - self.alpha_des23
        eta_alpha23 = Control1.z_barrier(self, z_alpha23, z_alpha23_bar, z_alpha23_under)
        d_eta_alpha23 = Control1.d_z_barrier(self, z_alpha23, z_alpha23_bar, z_alpha23_under)

        # 虚拟控制律
        c_2 = np.dot(np.linalg.inv(H_2), (-(self.k_rho2 * eta_rho_2 * d_eta_rho_2 * psi_2) +
                                          bar_psi_2 * (-self.k_alpha_2 *
                                                       (eta_alpha23 * d_eta_alpha23 * (
                                                               beta_alpha23 * rho_2)) +
                                                       e_alpha_23 * d_beta_alpha23 * rho_2 / beta_alpha23) + v02_hat))
        S_2 = eta_rho_2 * d_eta_rho_2 * psi_2.T + 1 / rho_2 * (eta_alpha23 * d_eta_alpha23) * bar_psi_2.T
        d_v02_hat = np.dot(self.Gamma, (-S_2.T - np.dot(self.K_0, v02_hat)))

        # 侧滑角观测器
        d_hat_dot_beta_angle_2 = -self.k_beta2 * self.hat_dot_beta_angle_2
        self.hat_dot_beta_angle_2 = 0.01 * d_hat_dot_beta_angle_2 + self.hat_dot_beta_angle_2
        # self.hat_dot_beta_angle = 0

        alpha_usv_U2 = c_2[0][0]
        alpha_usv_w2 = c_2[1][0]
        # 限幅
        # alpha_usv_U2 = speed__limit_proportion(alpha_usv_U2, 0)
        # alpha_usv_w2 = speed__limit_proportion(alpha_usv_w2, 1)

        e_phi_2 = (phi_2 - (math.pi / 2 + theta_2))  # 朝向角误差
        while abs(e_phi_2) >= math.pi:
            if e_phi_2 > 0:
                e_phi_2 = e_phi_2 - 2 * math.pi
            else:
                e_phi_2 = e_phi_2 + 2 * math.pi

        # agent1的状态
        usv1_location_x = x[0]  # agent的x位置
        usv1_location_y = x[1]  # agent的y位置
        usv1_course_angle = x[2] * math.pi / 180  # agent的朝向 转换成为弧度
        u_1 = x[3]
        v_1 = x[4]
        r_1 = x[5]
        v01_hat = np.array([[x[32]], [x[33]]])
        beta_angle_1 = math.atan2(v_1, u_1)
        #############################################
        # slide angle deal
        if abs(u_1) < 0.05 and abs(v_1) < 0.05:
            beta_angle_1 = 0
        #############################################
        usv_yaw_angle_plus_beta_1 = beta_angle_1 + usv1_course_angle

        H_1 = np.array([[math.cos(usv_yaw_angle_plus_beta_1), -self.vehicle_d * math.sin(usv1_course_angle)],
                        [math.sin(usv_yaw_angle_plus_beta_1),
                         self.vehicle_d * math.cos(usv1_course_angle)]])  # 旋转矩阵
        rho_1 = math.sqrt((math.pow(usv1_location_x - target_location_x, 2) + math.pow(usv1_location_y -
                                                                                       target_location_y, 2)))  # 计算两船距离
        e_rho_1 = rho_1 - self.r_des  # 围捕的距离误差
        print("agent1距离误差：" + str(e_rho_1))
        eta_rho_1 = Control1.barrier(self, e_rho_1)  # 障碍函数
        d_eta_rho_1 = Control1.d_barrier(self, e_rho_1)  # 障碍函数的导数
        p_1 = np.array([[usv1_location_x], [usv1_location_y]])  # 围捕船的位置向量，[x, y]^T
        p_0 = np.array([[target_location_x], [target_location_y]])  # Target的位置向量， [x_0, y_0]^T
        psi_1 = (p_1 - p_0) / rho_1  # Target与agent之间的连线的单位向量
        phi_1 = usv1_course_angle  # 方便移植，不然老是忘记这个是朝向
        theta_1 = math.atan2(psi_1[1], psi_1[0])  # Target与agent之间连线的方位角
        bar_psi_1 = np.array(
            [[math.cos(theta_1 + math.pi / 2)],
             [math.sin(theta_1 + math.pi / 2)]])  # 与psi角垂直的向量，-pi/2表示逆时针旋转,与MATLAB坐标系不同
        p_01 = p_1 - p_0
        p_02 = p_2 - p_0
        angle_alpha_12 = math.acos(np.dot(p_01.T, p_02) / (np.linalg.norm(p_01) * np.linalg.norm(p_02)))
        e_alpha_12 = angle_alpha_12 - self.alpha_des12
        beta_alpha12 = Control1.beta(self, current_time)
        d_beta_alpha12 = Control1.d_beta(self, current_time)
        z_alpha12 = e_alpha_12 / beta_alpha12
        z_alpha12_under = self.alpha_des12 - self.alpha_min
        z_alpha12_bar = self.alpha_max - self.alpha_des12
        eta_alpha12 = Control1.z_barrier(self, z_alpha12, z_alpha12_bar, z_alpha12_under)
        d_eta_alpha12 = Control1.d_z_barrier(self, z_alpha12, z_alpha12_bar, z_alpha12_under)

        # 虚拟控制律
        c_1 = np.dot(np.linalg.inv(H_1), (-(self.k_rho1 * eta_rho_1 * d_eta_rho_1 * psi_1) +
                                          bar_psi_1 * (-self.k_alpha_1 *
                                                       (eta_alpha12 * d_eta_alpha12 * (
                                                               beta_alpha12 * rho_1)) +
                                                       e_alpha_12 * d_beta_alpha12 * rho_1 / beta_alpha12) + v01_hat))
        S_1 = eta_rho_1 * d_eta_rho_1 * psi_1.T + 1 / rho_1 * (eta_alpha12 * d_eta_alpha12) * bar_psi_1.T
        d_v01_hat = np.dot(self.Gamma, (-S_1.T - np.dot(self.K_0, v01_hat)))

        # 侧滑角观测器
        d_hat_dot_beta_angle_1 = -self.k_beta1 * self.hat_dot_beta_angle_1
        self.hat_dot_beta_angle_1 = 0.01 * d_hat_dot_beta_angle_1 + self.hat_dot_beta_angle_1
        # self.hat_dot_beta_angle = 0

        alpha_usv_U1 = c_1[0][0]
        alpha_usv_w1 = c_1[1][0]
        # 限幅
        # alpha_usv_U1 = speed__limit_proportion(alpha_usv_U1, 0)
        # alpha_usv_w1 = speed__limit_proportion(alpha_usv_w1, 1)

        e_phi_1 = (phi_1 - (math.pi / 2 + theta_1))  # 朝向角误差
        while abs(e_phi_1) >= math.pi:
            if e_phi_1 > 0:
                e_phi_1 = e_phi_1 - 2 * math.pi
            else:
                e_phi_1 = e_phi_1 + 2 * math.pi

        # 第n艘与第1艘之间的夹角
        p_05 = p_5 - p_0
        angle_alpha_51 = math.acos(np.dot(p_05.T, p_01) / (np.linalg.norm(p_05) * np.linalg.norm(p_01)))
        e_alpha_51 = angle_alpha_51 - (math.pi * 2 - self.alpha_des12 - self.alpha_des23 - self.alpha_des34 -
                                       self.alpha_des45)

        xdot = [c_1[0][0], 0, c_1[1][0], d_v01_hat[0][0], d_v01_hat[1][0], e_rho_1, e_phi_1,
                e_alpha_12,
                c_2[0][0], 0, c_2[1][0], d_v02_hat[0][0], d_v02_hat[1][0], e_rho_2, e_phi_2,
                e_alpha_23,
                c_3[0][0], 0, c_3[1][0], d_v03_hat[0][0], d_v03_hat[1][0], e_rho_3, e_phi_3,
                e_alpha_34,
                c_4[0][0], 0, c_4[1][0], d_v04_hat[0][0], d_v04_hat[1][0], e_rho_4, e_phi_4,
                e_alpha_45,
                c_5[0][0], 0, c_5[1][0], d_v05_hat[0][0], d_v05_hat[1][0], e_rho_5, e_phi_5,
                e_alpha_51]

        return xdot

    def speed_thrust(self, alpha_1, alpha_2, velocity, usv_angle_speed):
        # “运动学，速度转推力”
        x_speed_desired = alpha_1[0]
        y_speed_desired = alpha_1[1]
        speed_desired = math.sqrt(math.pow(x_speed_desired, 2) + math.pow(y_speed_desired, 2))
        yaw_speed_desired = alpha_2
        speed_err = speed_desired - float(velocity)
        speed_pid = PID()
        thrust_total = speed_pid.calculate_pid(speed_err, 100, 1, 1)
        angle_speed_err = yaw_speed_desired - usv_angle_speed
        course_angle_pid = PID()
        thrust_left_right_diff = course_angle_pid.calculate_pid(angle_speed_err, 100, 1, 1)
        if angle_speed_err > 0:
            left_thrust = thrust_total / 2 + abs(thrust_left_right_diff) / 2
            right_thrust = thrust_total / 2 - abs(thrust_left_right_diff) / 2
        else:
            left_thrust = thrust_total / 2 - abs(thrust_left_right_diff) / 2
            right_thrust = thrust_total / 2 + abs(thrust_left_right_diff) / 2
        # # left_thrust, right_thrust = 0, 0
        # if float(velocity) > 1000:  # 设定速度阈值
        #     left_thrust = left_thrust - 200
        #     right_thrust = right_thrust - 200
        # elif float(course_angle) > 5 and float(endpoint_distance) > 1500:  # 边转向边前进，顺时针转
        #
        #     left_thrust = 100+left_thrust
        #     right_thrust = 50+right_thrust
        # elif float(course_angle) < -5 and float(endpoint_distance) > 1500:  # 逆时针转
        #     left_thrust = 50 + left_thrust
        #     right_thrust = 100 + right_thrust
        #
        # elif float(endpoint_distance) > 1500 and abs(float(course_angle)) < 5:  # 方向正确，加速前进
        #     left_thrust = 5000
        #     right_thrust = 5000
        #     time.sleep(0.05)  # 防止频繁启动
        # # elif float(endpoint_distance) < 1500 and abs(float(course_angle)) > 5:
        # #     left_thrust = 2000
        # #     right_thrust = -2000
        #
        # else:
        #     left_thrust = 0
        #     right_thrust = 0
        left_thrust = limit_thrust(left_thrust)
        right_thrust = limit_thrust(right_thrust)

        return left_thrust, right_thrust

    def dynamic_model_of_usv(self, u_1, v_1, r_1):
        d111 = 0.7225 + 1.3274 * abs(u_1) + 5.8664 * math.pow(u_1, 2)
        d221 = 0.8612 + 36.2823 * abs(v_1) + 0.805 * abs(r_1)
        d231 = -0.1079 + 0.845 * abs(v_1) + 3.45 * abs(r_1)
        d321 = -0.1052 - 5.0437 * abs(v_1) - 0.13 * abs(r_1)
        d331 = 1.9 - 0.08 * abs(v_1) + 0.75 * abs(r_1)

        # mass parameters
        m11 = 25.8
        m22 = 33.8
        m23 = 1.0948
        m33 = 2.76
        bar_m33 = m22 * m33 - math.pow(m23, 2)
        M = np.array([[m11, 0, 0], [0, m22, m23], [0, m23, m33]])
        inv_M = np.array([[1 / m11, 0, 0], [0, m33 / bar_m33, -m23 / bar_m33], [0, -m23 / bar_m33, m22 / bar_m33]])
        D1 = np.array([[d111, 0, 0], [0, d221, d231], [0, d321, d331]])
        C1 = np.array([[0, 0, -m22 * v_1 - m23 * r_1], [0, 0, m11 * u_1], [m22 * v_1 + m23 * r_1, -m11 * u_1, 0]])
        return M, inv_M, D1, C1, m11, m22, m23, bar_m33

    def barrier(self, e_rho):
        eta_rho = self.e_bar_rho * self.e_underline_rho * e_rho / (
                (e_rho + self.e_underline_rho) * (self.e_bar_rho - e_rho))
        return eta_rho

    def d_barrier(self, e_rho):
        d_eta_rho = self.e_bar_rho * self.e_underline_rho * (
                self.e_bar_rho * self.e_underline_rho + math.pow(e_rho, 2)) / \
                    math.pow((e_rho + self.e_underline_rho) * (self.e_bar_rho - e_rho),
                             2)
        return d_eta_rho

    def z_barrier(self, z_alpha, z_alpha_bar, z_alpha_under):
        eta_alpha = z_alpha_bar * z_alpha_under * z_alpha / ((z_alpha + z_alpha_under) * (z_alpha_bar - z_alpha))
        return eta_alpha

    def d_z_barrier(self, z_alpha, z_alpha_bar, z_alpha_under):
        d_eta_alpha = z_alpha_bar * z_alpha_under * (z_alpha_bar * z_alpha_under + math.pow(z_alpha, 2)) / \
                      math.pow((z_alpha + z_alpha_under) * (z_alpha_bar - z_alpha), 2)
        return d_eta_alpha

    def beta(self, current_time):
        beta_alpha = (self.beta_alpha_0 - self.beta_alpha_inf) * math.exp(
            -self.beta_gamma * current_time) + self.beta_alpha_inf
        return beta_alpha

    def d_beta(self, current_time):
        d_beta_alpha = -self.beta_gamma * (self.beta_alpha_0 - self.beta_alpha_inf) * math.exp(
            -self.beta_gamma * current_time)
        return d_beta_alpha

    def control_follow(self, course_angle, endpoint_distance, velocity, left_thrust, right_thrust, desired_speed,
                       course_angle_leader):
        if float(endpoint_distance) < 100:
            return 0, 0
        speed_err = desired_speed - float(velocity)
        speed_pid = PID()
        thrust_total = speed_pid.calculate_pid(speed_err, 100, 100, 20)
        course_angle_pid = PID()
        thrust_left_right_diff = course_angle_pid.calculate_pid(course_angle_leader, 1000, 20, 50)
        if course_angle > 0:
            left_thrust = thrust_total / 2 + abs(thrust_left_right_diff) / 2
            right_thrust = thrust_total / 2 - abs(thrust_left_right_diff) / 2
        else:
            left_thrust = thrust_total / 2 - abs(thrust_left_right_diff) / 2
            right_thrust = thrust_total / 2 + abs(thrust_left_right_diff) / 2
        left_thrust = limit_thrust(left_thrust)
        right_thrust = limit_thrust(right_thrust)
        return left_thrust, right_thrust


class PID:
    def __init__(self):
        self.data = {'last_p': 0, 'last_i': 0, 'last_d': 0, 'err_i': 0.0, 'last_err': 0.0, 'period': 0.01, ' p': 0,
                     'i': 0, 'd': 0}

    def calculate_pid(self, err, p, i, d):
        if abs(self.data['last_p'] - p) > 0.001 or abs(self.data['last_i'] - i) > 0.001 or abs(
                self.data['last_d'] - d) > 0.001:
            self.data['err_i'] = 0

        self.data['err_i'] += err * self.data['period']
        err_d = (err - self.data['last_err']) / self.data['period']

        self.data['last_p'] = p
        self.data['last_i'] = i
        self.data['last_d'] = d
        self.data['last_err'] = err

        return err * p + self.data['err_i'] * i + err_d * d
