#!/usr/bin/env python
import rospy
import copy
import tf
import math


from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from std_msgs.msg import Int8


# 속도 정보 추가된 템플릿
CAR_TEMPLATE = {'state': 0, 'x': 0, 'y': 0, 't': 0, 'v': 0, 'path': [], 'velocity_ms': 0}

class ROSManager:
    def __init__(self, type, targets_num):
        self.type = type
        self.targets_num = targets_num
        self.set_values()
        self.set_protocol()
    
    def set_values(self):
        self.ego = copy.deepcopy(CAR_TEMPLATE)
        self.targets = []
        for i in range(self.targets_num):
            self.targets.append(copy.deepcopy(CAR_TEMPLATE))

        self.target_velocity = 30/3.6

    def set_protocol(self):
        # ego 구독 - odom으로 변경
        rospy.Subscriber('/ego/odom', Odometry, 
                        lambda msg: self.odom_callback(msg, self.ego))
        rospy.Subscriber('/ego/path', Path, 
                        lambda msg: self.path_callback(msg, self.ego))
        rospy.Subscriber('/ego/car_state', Int8, 
                        lambda msg: self.car_state_callback(msg, self.ego))
        
        # targets 구독 - odom으로 변경
        for i in range(self.targets_num):
            rospy.Subscriber(f'/target{i+1}/odom', Odometry, 
                            lambda msg, idx=i: self.odom_callback(msg, self.targets[idx]))
            rospy.Subscriber(f'/target{i+1}/path', Path, 
                            lambda msg, idx=i: self.path_callback(msg, self.targets[idx]))
            rospy.Subscriber(f'/target{i+1}/car_state', Int8, 
                            lambda msg, idx=i: self.car_state_callback(msg, self.targets[idx]))
            
        self.lh_test_pub = rospy.Publisher(f'/ego/look_a_head', Marker, queue_size=1)
            
    def odom_callback(self, msg, car_dict):
        """Odometry 콜백 - 위치, 자세, 속도 정보 모두 업데이트"""
        # 위치 정보
        car_dict['x'] = msg.pose.pose.position.x
        car_dict['y'] = msg.pose.pose.position.y
        
        # 자세 정보 (yaw 각도)
        quaternion = (
            msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z, 
            msg.pose.pose.orientation.w
        )
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        car_dict['t'] = math.degrees(yaw)  # 기존 코드와 호환성 위해 degrees 유지
        
        # 속도 정보 (m/s)
        # linear.x는 차량 전진 방향 속도
        car_dict['velocity_ms'] = msg.twist.twist.linear.x
        
        # 기존 'v' 필드도 유지 (호환성)
        car_dict['v'] = msg.twist.twist.linear.x

    def path_callback(self, msg, car_dict):
        """공통 path 콜백"""
        car_dict['path'] = [(p.pose.position.x, p.pose.position.y) 
                            for p in msg.poses]

    def car_state_callback(self, msg, car_dict):
        """차량 상태 콜백"""
        car_dict['state'] = int(msg.data)
        
    def get_ego_data(self):
        """ego 차량 데이터 반환"""
        return copy.deepcopy(self.ego)
    
    def get_target_data(self, target_idx):
        """특정 target 차량 데이터 반환"""
        if 0 <= target_idx < len(self.targets):
            return copy.deepcopy(self.targets[target_idx])
        else:
            rospy.logwarn(f"Invalid target index: {target_idx}")
            return None
    
    def get_all_targets_data(self):
        """모든 target 차량 데이터 반환"""
        return [copy.deepcopy(target) for target in self.targets]
    
    def get_vehicle_speed_ms(self, vehicle_type='ego', target_idx=0):
        """차량 속도 (m/s) 반환"""
        if vehicle_type == 'ego':
            return self.ego['velocity_ms']
        elif vehicle_type == 'target' and 0 <= target_idx < len(self.targets):
            return self.targets[target_idx]['velocity_ms']
        else:
            return 0.0
    
    def get_vehicle_speed_kmh(self, vehicle_type='ego', target_idx=0):
        """차량 속도 (km/h) 반환"""
        speed_ms = self.get_vehicle_speed_ms(vehicle_type, target_idx)
        return speed_ms * 3.6
    
    def is_vehicle_moving(self, vehicle_type='ego', target_idx=0, threshold=0.1):
        """차량이 움직이고 있는지 확인 (threshold: m/s)"""
        speed = self.get_vehicle_speed_ms(vehicle_type, target_idx)
        return abs(speed) > threshold

    def pub_lh(self, lh):
        """Look-ahead 포인트 마커 발행"""
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'lookahead'
        marker.id = 1
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 0.7
        marker.scale.y = 0.7
        marker.scale.z = 0.7
        
        # 차량 타입에 따른 색상 설정
        if self.type == 'ego':
            marker.color.r = 241/255.0
            marker.color.g = 76/255.0
            marker.color.b = 152/255.0
        else:
            marker.color.r = 94/255.0
            marker.color.g = 204/255.0
            marker.color.b = 243/255.0
        marker.color.a = 1.0
        
        marker.pose.position.x = lh[0]
        marker.pose.position.y = lh[1]
        marker.pose.position.z = 1.0
        
        # 기본 orientation 설정
        marker.pose.orientation.w = 1.0
        
        self.lh_test_pub.publish(marker)
    
    def print_vehicle_status(self):
        """차량 상태 정보 출력 (디버깅용)"""
        print(f"\n=== Vehicle Status ===")
        print(f"EGO: pos=({self.ego['x']:.2f}, {self.ego['y']:.2f}), "
              f"yaw={self.ego['t']:.1f}°, speed={self.ego['velocity_ms']:.2f}m/s, "
              f"state={self.ego['state']}")
        
        for i, target in enumerate(self.targets):
            print(f"TARGET{i+1}: pos=({target['x']:.2f}, {target['y']:.2f}), "
                  f"yaw={target['t']:.1f}°, speed={target['velocity_ms']:.2f}m/s, "
                  f"state={target['state']}")
    
    def cleanup(self):
        """정리 작업"""
        rospy.loginfo("ROSManager cleanup completed")