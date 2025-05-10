#  @Time : 2023/06/18
#  @Author: adi
import numpy as np


def deal(data):  # 处理TCP通信中接收的数据
    course_angle_float = 0
    data = data.split(',')
    usv3_velocity_x = data[0]
    usv3_velocity_y = data[1]
    usv3_location_x = data[2]
    usv3_location_y = data[3]
    usv3_course_angle = data[4]
    usv3_velocity = data[5]
    usv3_course_angle_speed = data[6]
    usv2_velocity_x = data[7]
    usv2_velocity_y = data[8]
    usv2_location_x = data[9]
    usv2_location_y = data[10]
    usv2_course_angle = data[11]
    usv2_velocity = data[12]
    usv2_course_angle_speed = data[13]
    usv1_velocity_x = data[14]
    usv1_velocity_y = data[15]
    usv1_location_x = data[16]
    usv1_location_y = data[17]
    usv1_course_angle = data[18]
    usv1_velocity = data[19]
    usv1_course_angle_speed = data[20]
    target_x = data[21]
    target_y = data[22]
    usv4_velocity_x = data[23]
    usv4_velocity_y = data[24]
    usv4_location_x = data[25]
    usv4_location_y = data[26]
    usv4_course_angle = data[27]
    usv4_velocity = data[28]
    usv4_course_angle_speed = data[29]
    usv5_velocity_x = data[30]
    usv5_velocity_y = data[31]
    usv5_location_x = data[32]
    usv5_location_y = data[33]
    usv5_course_angle = data[34]
    usv5_velocity = data[35]
    usv5_course_angle_speed = data[36]
    # course_angle_float = float(course_angle)

    return usv1_velocity_x, usv1_velocity_y, usv1_location_x, usv1_location_y, usv1_course_angle, usv1_velocity, \
        usv1_course_angle_speed, usv2_velocity_x, usv2_velocity_y, usv2_location_x, usv2_location_y, usv2_course_angle,\
        usv2_velocity, usv2_course_angle_speed, usv3_velocity_x, usv3_velocity_y, usv3_location_x, usv3_location_y, \
        usv3_course_angle, usv3_velocity, usv3_course_angle_speed, target_x, target_y, usv4_velocity_x, usv4_velocity_y,\
        usv4_location_x, usv4_location_y, usv4_course_angle, usv4_velocity, usv4_course_angle_speed, usv5_velocity_x, \
        usv5_velocity_y, usv5_location_x, usv5_location_y, usv5_course_angle, usv5_velocity, usv5_course_angle_speed
