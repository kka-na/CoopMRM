#!/usr/bin/env python3
import rospy
import sys
import signal
import asyncio

from ros_manger import ROSManager
from control.control import Control
from transmitter.main import Transmitter
import math


def signal_handler(sig, frame):
    sys.exit(0)

def upsample_path_1m(limited_path):
    """
    limited_path: (e, n) 좌표 튜플들의 리스트 (다운샘플된 경로)
    반환: 각 선분을 1m 간격으로 보간하여 생성된 경로
    """
    if not limited_path:
        return []
    
    upsampled = [limited_path[0]]
    
    for i in range(1, len(limited_path)):
        start = limited_path[i-1]
        end = limited_path[i]
        
        # 선분 길이 계산 (유클리드 거리)
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        segment_length = math.sqrt(dx**2 + dy**2)
        
        # 선분 길이가 1m보다 크다면, 1m 간격 점 추가 (선형 보간)
        num_points = int(segment_length)  # 정수 부분만큼 1m 간격 점이 들어갈 수 있음
        
        for j in range(1, num_points + 1):
            distance = j  # 시작점으로부터의 거리 (1, 2, 3, ... m)
            # segment_length보다 작은 거리까지만 보간 (endpoint는 마지막에 추가)
            if distance < segment_length:
                t = distance / segment_length
                new_point = (start[0] + dx * t, start[1] + dy * t)
                upsampled.append(new_point)
        
        if upsampled[-1] != end:
            upsampled.append(end)
    
    return upsampled

class Execution():
    def __init__(self, type,car, map):
        self.RM = ROSManager(type)
        self.ct = Control(car)
        self.tm = Transmitter(type, car, map)
        self.set_values()
    
    def set_values(self):
        self.car = None
        self.local_path = None

    def update_values(self):
        self.car = self.RM.car
        self.local_path = upsample_path_1m(self.RM.local_path)
        self.ct.update_value(self.RM.target_velocity, self.car, self.local_path)


    async def control(self):
        while not rospy.is_shutdown():
            self.update_values()
            actuator, lh = self.ct.execute()
            #self.RM.pub_lh(lh)
            self.tm.target.set_actuator(actuator)
            self.tm.target.set_user_input(self.RM.user_input)
            await asyncio.sleep(0.1) #10hz             

    def execute(self):
        loop = asyncio.get_event_loop()
        control = loop.create_task(self.control())
        transmitter = loop.create_task(self.tm.transmitter())
        loop.run_forever()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    if len(sys.argv) != 4:
        type = 'ego'
        car = 'simulator'
        map = 'Midan'
    else:
        type = str(sys.argv[1])
        car = str(sys.argv[2])
        map = str(sys.argv[3])
    
    ex = Execution(type, car, map)
    ex.execute()

if __name__ == "__main__":
    main()