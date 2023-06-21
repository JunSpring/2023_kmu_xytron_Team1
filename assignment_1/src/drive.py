#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 필요한 라이브러리와 메시지 타입을 가져옵니다.
import rospy, math
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

# "Avoid"라는 클래스를 생성합니다.
class Avoid:
    def __init__(self):
        # 초음파 데이터를 수신받았는지 여부를 추적하기 위한 변수를 초기화합니다.
        self.b_ultrasonic = False
        
        # "ultrasonic" 토픽을 구독하고 콜백 함수를 정의합니다.
        self.ultrasonic_subscriber = rospy.Subscriber("ultrasonic", Int32MultiArray, self.ultrasonic_CB)
        
        # "xycar_motor" 토픽에 발행합니다.
        self.motor_publisher = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        
        # xycar_motor 메시지의 인스턴스를 생성합니다.
        self.data = xycar_motor()

        # 속도와 각도 변수를 초기화합니다.
        self.speed = 0 
        self.angle = 0

    # "ultrasonic" 토픽의 콜백 함수입니다.
    def ultrasonic_CB(self, msg):
        # 초음파 데이터를 처음 받았을 때 로그 메시지를 출력합니다.
        if not self.b_ultrasonic:
            rospy.loginfo("초음파 데이터 수신 완료")
            self.b_ultrasonic = True

        # 수신한 초음파 데이터를 딕셔너리로 변환합니다.
        self.ultra_data = self.swap_list_to_dic(msg.data)
        
        # 데이터를 분석하고 xycar_motor 메시지에 적절한 값을 설정합니다.
        self.data_anlyze() 

    # 데이터를 분석하고 xycar_motor 메시지에 값을 설정하는 함수입니다.
    def data_anlyze(self):
        # DR과 DL의 차이를 계산합니다.
        dif = self.ultra_data["DR"] - self.ultra_data["DL"]
        
        # 차이를 6으로 나눈 값을 각도로 설정합니다.
        self.data.angle = dif // 6
        
        # 속도를 1000으로 설정합니다.
        self.data.speed = 1000
        
        # 각도에 따라 로그 메시지를 출력합니다.
        if self.data.angle < 0:
            rospy.loginfo(f"왼쪽: {self.data.angle}")
        elif int(self.data.angle) == 0:
            rospy.loginfo(f"직진: {self.data.angle}")
        else:
            rospy.loginfo(f"오른쪽: {self.data.angle}")        

        # xycar_motor 메시지를 발행합니다.
        self.motor_publisher.publish(self.data)
    
    # 리스트를 딕셔너리로 변환하는 함수입니다.
    def swap_list_to_dic(self, data):
        # "DL과 DR의 차이 존재 -> DL에 offset 1 추가"
        return {"L": data[0], "DL": data[1] + 1, "C": data[2], "DR": data[3], "R": data[4]}

  
if __name__=="__main__":
    # Avoid 클래스의 인스턴스를 생성합니다.
    AV = Avoid()
    
    # 노드를 초기화합니다.
    rospy.init_node('driver')
    
    # 노드가 종료될 때까지 실행을 계속합니다.
    rospy.spin()
