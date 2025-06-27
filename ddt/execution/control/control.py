import time
import sys
import signal
import numpy as np
import configparser

from control.libs.apid import APID
from control.libs.purepursuit import PurePursuit
from control.libs.point import Point 

# 속도 제어 상수
PATH_LENGTH_THRESHOLD = 50  # meter, 가속 가능한 최소 경로 길이
ACCELERATION_AGGRESSIVE = 1.0  # m/s, 큰 속도 차이시 가속도
ACCELERATION_CONSERVATIVE = 0.5  # m/s, 작은 속도 차이시 가속도
DECELERATION_RATE = 0.5  # m/s, 경로 부족시 감속도
VELOCITY_DIFF_THRESHOLD_RATIO = 0.5  # 최대속도의 50%

def signal_handler(sig, frame):
    sys.exit(0)

class Control():
    def __init__(self, car):
        config = self.get_config(car)
        self.APID = APID(config)
        self.PP = PurePursuit(config)
        self.set_values()
    
    def get_config(self, car):
        config_file_path = f'./control/configs/{car}.ini'
        config = configparser.ConfigParser()
        config.read(config_file_path)
        return config
    
    def set_values(self):
        """제어 변수 초기화"""
        self.max_velocity = 0.0
        self.target_velocity = 0.0
        self.state = 0
        self.current_location = Point(x=0, y=0)
        self.current_velocity = 0.0
        self.current_heading = 0.0
        self.local_path = []
        
    def update_value(self, max_velocity, _car):
        """외부에서 받은 정보로 제어 변수 업데이트"""
        self.max_velocity = max_velocity
        self.state = _car['state']
        self.current_location = Point(x=_car['x'], y=_car['y'])
        self.current_velocity = _car['v']
        self.current_heading = _car['t']
        
        # 경로 데이터 변환
        self.local_path = []
        if _car['path'] is not None:
            for point in _car['path']:
                self.local_path.append(Point(x=point[0], y=point[1]))
        
        # 경로 길이 기반 목표 속도 계산
        path_length = len(_car['path']) if _car['path'] is not None else 0
        self.target_velocity = self._calculate_target_velocity(path_length)
            
    def _calculate_target_velocity(self, path_length):
        """경로 길이와 현재 상태 기반 목표 속도 계산"""
        # 경로가 충분히 긴 경우 - 가속 가능
        if path_length > PATH_LENGTH_THRESHOLD:
            return self._calculate_acceleration_velocity()
        else:
            # 경로가 짧거나 없는 경우 - 감속 필요
            return self._calculate_deceleration_velocity()

    def _calculate_acceleration_velocity(self):
        """가속 상황에서의 목표 속도 계산"""
        velocity_difference = self.max_velocity - self.current_velocity
        velocity_threshold = self.max_velocity * VELOCITY_DIFF_THRESHOLD_RATIO
        
        # 현재 속도와 최대 속도의 차이에 따라 가속도 조절
        if velocity_difference > velocity_threshold:
            # 속도 차이가 큰 경우 - 적극적 가속
            target = self.current_velocity + ACCELERATION_AGGRESSIVE
        else:
            # 속도 차이가 작은 경우 - 보수적 가속
            target = self.current_velocity + ACCELERATION_CONSERVATIVE
        
        # 최대 속도 제한
        return min(target, self.max_velocity)

    def _calculate_deceleration_velocity(self):
        """감속 상황에서의 목표 속도 계산"""
        target = self.current_velocity - DECELERATION_RATE
        
        # 음수 속도 방지
        return max(target, 0.0)

    def execute(self):
        """제어 알고리즘 실행"""
        # 종방향 제어 (속도)
        acceleration_command = self.APID.execute(
            self.state, 
            self.target_velocity, 
            self.current_velocity
        )
        
        # 횡방향 제어 (조향)
        steering_command, lookahead_point = self.PP.execute(
            self.current_location, 
            self.state, 
            self.local_path, 
            self.current_heading, 
            self.current_velocity
        )
        
        return [acceleration_command, steering_command], lookahead_point

    def get_control_status(self):
        """현재 제어 상태 정보 반환 (디버깅용)"""
        return {
            'current_velocity': self.current_velocity,
            'target_velocity': self.target_velocity,
            'max_velocity': self.max_velocity,
            'path_length': len(self.local_path),
            'state': self.state,
            'current_location': (self.current_location.x, self.current_location.y),
            'current_heading': self.current_heading
        }
        
def main():
    signal.signal(signal.SIGINT, signal_handler)
    control = Control()
    control.execute()

if __name__ == "__main__":
    main()