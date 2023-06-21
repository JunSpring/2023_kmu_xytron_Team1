#!/usr/bin/env python
#-- coding:utf-8 --

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
import math
import rospy
from xycar_msgs.msg import xycar_motor

# reeds_shepp.py를 rs로 불러옴
import reeds_shepp as rs
# utils.py의 함수를 불러옴
from utils import *

#=============================================
# 모터 토픽을 발행할 것임을 선언
#============================================= 
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#============================================= 
rx, ry = [300, 350, 400, 450], [900, 850, 800, 700]

# Global Variables
path = None                 # 전체 경로
end = None                  # 진입로 좌표
current_path = None         # 현재 좌표
angle, speed = None, None   # 주행 각도 및 속도
tracking_distance = 0       # 한 경로의 남은 거리

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
def make_path(start, end):
    """
    start의 좌표(각도 포함)에서 end의 좌표(각도 포함)까지의
    Reeds Shepp Algorithm을 사용한 경로를 반환해주는 함수
    """

    # Global Variables
    global tracking_distance
    global path

    # Print
    print("Make Path!")

    # Save Data
    x = start[0]
    y = start[1]
    yaw = start[2]

    # Change Coordinate
    start[0] = x+64*math.cos(deg2rad(yaw+90.0))
    start[1] = y+64*math.sin(deg2rad(yaw+90.0))

    change_of_coordinate(start)
    
    # Get Optimal Path
    path = rs.get_optimal_path(start, end)

    # Init Tracking Distance
    tracking_distance = 0
    
    # Print Path
    for e in path:
        print(e)

    # Return Path
    return path

def planning(sx, sy, syaw, max_acceleration, dt):
    """
    Planning버튼을 누를 시 호출되는 함수
    처음 위치에서 진입 시까지의 Path를 저장함
    rx와 ry를 반환함
    """

    # Global Variables
    global rx, ry
    global path, end

    # Print
    print("Start Planning")

    # Save in start
    start = [sx, sy, syaw]
    
    """
    정방향으로 진입할 때와 역방향으로 진입할 때의 Path를 비교하여
    가장 효율적인 Path를 선택함
    """

    # Test Forward Path
    end_forward = [P_ENTRY[0], P_ENTRY[1], 225.0]  
    change_of_coordinate(end_forward)
    path_forward = make_path(start, end_forward)

    # Test Backward Path
    end_backward = [P_ENTRY[0]-64*math.cos(deg2rad(45.0+90.0)), P_ENTRY[1]-64*math.sin(deg2rad(45.0+90.0)), 45.0]  # 종료 위치 (x, y, theta)
    change_of_coordinate(end_backward)
    path_backward = make_path(start, end_backward)

    # Compare Forward and Backward Path
    if rs.path_length(path_forward) <= rs.path_length(path_backward):
        # Set Forward Path
        path = path_forward
        end = end_forward
        print("Set Forward!")
    else:
        # Set Backward Path
        path = path_backward
        end = end_backward
        print("Set Backward!")

    # Print
    print(start, end)

    # Draw Path Failed
    # 경로를 그리는 것은 실패하였음
    # rx, ry = draw_path(start, end)

    # Return rx and ry
    return rx, ry

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    """
    Tracking버튼을 누를 시 주행하는 함수
    """
    
    # Global Variables
    global path, end
    global current_path
    global angle, speed
    global tracking_distance

    # current coordinate
    current = [x,y,360.0-yaw-90.0]

    # 진입에 성공했을 시
    if len(path) == 0 and tracking_distance <= 0:
        # Print
        print(dis(current, [end[0], -end[1], end[2]]))

        # Set Parameters
        angle = 0
        p_min = 64
        p_max = 66
        speed = 0

        # Final Entry
        if not p_min < dis(current, [end[0], -end[1], end[2]]) < p_max:
            gear1 = 1 if dis(current, [end[0], -end[1], end[2]]) <= p_min else -1
            gear2 = 1 if end[0] - current[0] > 64 else -1
            speed = 50 if end[2]==225.0 else -50

            speed *= gear1*gear2
        else:
            speed = 0

    # 아직 진입 전일 때
    else:
        """
        Path의 정확도를 위해서 100픽셀마다
        Path를 재생성하여 주행함
        """
        # Make Path on every 100 pixel
        if dis(current, P_ENTRY) > 100 and dis(current, P_ENTRY)%100 < 10:
            path = make_path(current, end)
        
        # Discount tracking_distance
        if tracking_distance > 0:
            tracking_distance -= dt * abs(velocity)
        else:
            current_path = None
            angle = None
            speed = None

        # Pop Path
        if current_path is None and len(path) != 0:
            current_path = path.pop(0)
            print("pop!")
        
        # Drive
        if angle is None or speed is None:
            speed = 50 if current_path.gear == rs.Gear.FORWARD else -50
            print("set speed!")
            if current_path.steering == rs.Steering.LEFT:
                angle = -20
                tracking_distance = 230*current_path.param
                print("set left!")
            elif current_path.steering == rs.Steering.RIGHT:
                angle = 20
                tracking_distance = 230*current_path.param
                print("set right!")
            else:
                angle = 0
                tracking_distance = current_path.param
                print("set straight!")

    drive(angle, speed)

