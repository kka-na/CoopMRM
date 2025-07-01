import numpy as np
import math

class PurePursuit(object):
    def __init__(self, config):
        self.set_configs(config)
        self.prev_steer = 0

    def set_configs(self, config):
        pp_config = config['PurePursuit']
        self.lfd_gain = float(pp_config['lfd_gain'])
        self.min_lfd = float(pp_config['min_lfd'])
        self.max_lfd = float(pp_config['max_lfd'])
        self.lfd_offset = float(pp_config['lfd_offset'])
        self.speed_threshold = float(pp_config['speed_threshold'])
        self.speed_gain = float(pp_config['speed_gain'])
        self.offset_base = float(pp_config['offset_base'])
        self.offset_min = float(pp_config['offset_min'])
        self.offset_max = float(pp_config['offset_max'])
        
        cm_config = config['Common']
        self.wheelbase = float(cm_config['wheelbase'])
        self.steer_ratio = float(cm_config['steer_ratio'])
        self.steer_max = float(cm_config['steer_max'])
        self.saturation_th = float(cm_config['saturation_th'])

    def execute(self, current_location, state, local_path, current_heading, current_velocity):
        if len(current_location) < 1:
            return 0, [current_location.x, current_location.y]
        
        # Look-ahead distance 계산 (속도 기반, MPS 단위로 직접 계산)
        lfd = self.lfd_gain * current_velocity + self.lfd_offset
        lfd = np.clip(lfd, self.min_lfd, self.max_lfd)

        point = current_location
        route = local_path
        heading = math.radians(current_heading)
        
        steering_angle = 0.
        lh_point = point

        # Pure Pursuit 알고리즘: 전방 탐색점 찾기
        closest_forward_point = None
        closest_distance = 0
        
        for i, path_point in enumerate(route):
            diff = path_point - point
            rotated_diff = diff.rotate(-heading)
            if rotated_diff.x > 0:  # 전방 점만 고려
                # 현재 차량 위치로부터의 실제 거리 계산
                dis = np.linalg.norm(diff)  # 또는 path_point.distance(point)
                
                # LFD 이상의 점을 찾으면 즉시 선택
                if dis >= lfd:
                    theta = rotated_diff.angle
                    steering_angle = np.arctan2(2 * self.wheelbase * np.sin(theta), lfd)
                    lh_point = path_point
                    break
                # LFD 미만이지만 전방에서 가장 먼 점 기록
                elif dis > closest_distance:
                    closest_distance = dis
                    closest_forward_point = path_point
        else:
            # LFD 이상의 점이 없다면 전방에서 가장 먼 점 사용
            if closest_forward_point is not None:
                diff = closest_forward_point - point
                rotated_diff = diff.rotate(-heading)
                theta = rotated_diff.angle
                # 실제 거리를 사용하여 조향각 계산
                actual_distance = np.linalg.norm(diff)
                steering_angle = np.arctan2(2 * self.wheelbase * np.sin(theta), max(actual_distance, 1.0))
                lh_point = closest_forward_point

        # 라디안을 도(degree)로 변환
        steering_angle = math.degrees(steering_angle)
        
        # 기본 조향각 제한 (설정값 사용)
        steering_angle = np.clip(steering_angle, -self.steer_max, self.steer_max)
        
        # 고속에서의 조향각 보정
        current_speed_kph = current_velocity * 3.6  # MPS to KPH 변환을 한 번만
        if current_speed_kph > self.speed_threshold:
            steer_offset = np.clip(
                current_speed_kph * self.speed_gain + self.offset_base,
                self.offset_min,
                self.offset_max
            )
            steering_angle = steering_angle * steer_offset
        
        # 최종 조향 명령 계산 (조향비 적용)
        steer = np.clip(steering_angle * self.steer_ratio, -500, 500)
        
        return steer, (lh_point.x, lh_point.y)