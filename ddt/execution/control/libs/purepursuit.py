import numpy as np
import math

# 상수 정의
MPS_TO_KPH = 3.6
BASIC_STEERING_LIMIT_DEG = 15.0
HIGH_SPEED_THRESHOLD_KPH = 30.0

# 고속 보정 계수
SPEED_OFFSET_GAIN = 0.02
SPEED_OFFSET_BASE = 0.1
SPEED_OFFSET_MIN = 1.0
SPEED_OFFSET_MAX = 2.2

class PurePursuit(object):
    def __init__(self, config):
        self.set_configs(config)

    def set_configs(self, config):
        pp_config = config['PurePursuit']
        self.lfd_gain = float(pp_config['lfd_gain'])
        self.min_lfd = float(pp_config['min_lfd'])
        self.max_lfd = float(pp_config['max_lfd'])
        self.lfd_offset = float(pp_config['lfd_offset'])
        
        common_config = config['Common']
        self.wheelbase = float(common_config['wheelbase'])
        self.steer_ratio = float(common_config['steer_ratio'])
        self.steer_max = float(common_config['steer_max'])

    def execute(self, current_location, state, local_path, current_heading, current_velocity):
        """Pure Pursuit 알고리즘 실행"""
        # 위치 정보가 없으면 조향각 0 반환
        if len(current_location) < 1:
            return 0, [current_location.x, current_location.y]
        
        # 속도 기반 lookahead distance 계산
        lookahead_distance = self._calculate_lookahead_distance(current_velocity)
        
        # 목표점 찾기
        target_point = self._find_target_point(
            current_location, local_path, current_heading, lookahead_distance
        )
        
        if target_point is None:
            return 0, [current_location.x, current_location.y]
        
        # 조향각 계산
        steering_angle = self._calculate_steering_angle(
            current_location, target_point, current_heading, lookahead_distance
        )
        
        # 속도 기반 조향각 보정
        corrected_steering = self._apply_speed_compensation(steering_angle, current_velocity)
        
        # 최종 조향 명령 생성
        final_steer_command = self._convert_to_steer_command(corrected_steering)
        
        return final_steer_command, (target_point.x, target_point.y)

    def _calculate_lookahead_distance(self, current_velocity):
        """속도에 기반한 lookahead distance 계산"""
        velocity_kmh = current_velocity * MPS_TO_KPH
        lfd = self.lfd_gain * velocity_kmh
        return np.clip(lfd, self.min_lfd, self.max_lfd)

    def _find_target_point(self, current_location, local_path, current_heading, lookahead_distance):
        """lookahead distance 기반으로 목표점 찾기"""
        heading_rad = math.radians(current_heading)
        
        for path_point in local_path:
            # 현재 위치에서 경로점까지의 벡터
            diff_vector = path_point - current_location
            
            # 차량 좌표계로 변환 (heading 기준 회전)
            rotated_diff = diff_vector.rotate(-heading_rad)
            
            # 전방에 있는 점만 고려
            if rotated_diff.x > 0:
                distance = rotated_diff.distance()
                
                # lookahead distance 이상인 첫 번째 점을 목표점으로 선택
                if distance >= lookahead_distance:
                    return path_point
        
        # 조건에 맞는 점이 없으면 마지막 점 반환
        return local_path[-1] if local_path else None

    def _calculate_steering_angle(self, current_location, target_point, current_heading, lookahead_distance):
        """Pure Pursuit 기본 조향각 계산"""
        heading_rad = math.radians(current_heading)
        
        # 목표점까지의 벡터
        diff_vector = target_point - current_location
        rotated_diff = diff_vector.rotate(-heading_rad)
        
        # Pure Pursuit 공식
        theta = rotated_diff.angle
        steering_angle_rad = np.arctan2(
            2 * self.wheelbase * np.sin(theta), 
            lookahead_distance * self.lfd_offset
        )
        
        # 라디안을 도(degree)로 변환
        return math.degrees(steering_angle_rad)

    def _apply_speed_compensation(self, steering_angle, current_velocity):
        """속도에 따른 조향각 보정"""
        # 기본 조향각 제한 적용
        limited_steering = max(-BASIC_STEERING_LIMIT_DEG, 
                              min(steering_angle, BASIC_STEERING_LIMIT_DEG))
        
        # 고속에서만 추가 보정 적용
        velocity_kmh = current_velocity * MPS_TO_KPH
        if velocity_kmh > HIGH_SPEED_THRESHOLD_KPH:
            speed_compensation = self._calculate_speed_offset(velocity_kmh)
            return limited_steering * speed_compensation
        
        return limited_steering

    def _calculate_speed_offset(self, velocity_kmh):
        """고속 주행시 조향 보정 계수 계산"""
        # 속도에 비례한 보정 계수 계산
        # field test를 통해 튜닝된 파라미터들
        speed_offset = velocity_kmh * SPEED_OFFSET_GAIN + SPEED_OFFSET_BASE
        
        # 보정 계수 범위 제한
        return min(max(speed_offset, SPEED_OFFSET_MIN), SPEED_OFFSET_MAX)

    def _convert_to_steer_command(self, steering_angle_deg):
        """조향각(degree)을 최종 조향 명령으로 변환"""
        # 조향 기어비 적용
        steer_command = steering_angle_deg * self.steer_ratio
        
        # 하드웨어 한계값으로 클리핑
        return np.clip(steer_command, -self.steer_max * self.steer_ratio, 
                      self.steer_max * self.steer_ratio)