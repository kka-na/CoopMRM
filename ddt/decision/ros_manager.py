#!/usr/bin/env python
import rospy
import threading
from pyproj import Proj, Transformer

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2

class ROSManager:
    def __init__(self, type, map):
        self.type = type
        self.map = map
        self.discovered_vehicles = set()  # 발견된 vehicle들
        self.lock = threading.Lock()  # thread-safe를 위한 lock
        self.set_values()
        self.set_protocol()
        
        # 주기적으로 새로운 odom 토픽을 찾는 타이머
        self.discovery_timer = rospy.Timer(rospy.Duration(2.0), self.discover_new_vehicles)
    
    def set_values(self):
        proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=self.map.base_lla[0], lon_0=self.map.base_lla[1], h_0=self.map.base_lla[2])
        self.geo2enu_transformer = Transformer.from_proj(proj_wgs84, proj_enu)
        self.enu2geo_transformter = Transformer.from_proj(proj_enu, proj_wgs84)

        self.current_poses = {}    # {vehicle_id: [x, y]}
        self.current_velocities = {}  # {vehicle_id: [vx, vy, angular_z]}
        self.subscribers = {}      # {vehicle_id: subscriber}
        self.path_pubs = {}        # {vehicle_id: publisher}

    def set_protocol(self):
        # map publish
        pointcloud_pub = rospy.Publisher('/map/combined', PointCloud2, queue_size=1, latch=True)
        pointcloud_pub.publish(self.map.lmap_viz)

        # ego는 기본으로 추가
        self.add_vehicle('ego')

    def discover_new_vehicles(self, event):
        """주기적으로 새로운 odom 토픽을 찾아서 추가"""
        try:
            # 현재 발행되고 있는 모든 토픽 목록 가져오기
            topics = rospy.get_published_topics()
            
            for topic_name, topic_type in topics:
                # /xxx/odom 패턴의 토픽 찾기 (Odometry 타입)
                if topic_name.endswith('/odom') and topic_type == 'nav_msgs/Odometry':
                    # vehicle_id 추출 (예: /target1/odom -> target1)
                    vehicle_id = topic_name.split('/')[1]
                    
                    # 아직 발견되지 않은 vehicle이면 추가
                    if vehicle_id not in self.discovered_vehicles:
                        rospy.loginfo(f"New vehicle discovered: {vehicle_id}")
                        self.add_vehicle(vehicle_id)
                        
        except Exception as e:
            rospy.logwarn(f"Error during vehicle discovery: {e}")

    def add_vehicle(self, vehicle_id):
        """새로운 vehicle을 동적으로 추가"""
        with self.lock:
            if vehicle_id in self.discovered_vehicles:
                return
                
            # Subscriber 추가 - odom으로 변경
            topic_name = f'/{vehicle_id}/odom'
            self.subscribers[vehicle_id] = rospy.Subscriber(
                topic_name, 
                Odometry, 
                lambda msg, vid=vehicle_id: self.odom_callback(msg, vid)
            )
            
            # Publisher 추가
            path_topic = f'/{vehicle_id}/path'
            self.path_pubs[vehicle_id] = rospy.Publisher(path_topic, Path, queue_size=1)
            
            # 발견된 vehicle 목록에 추가
            self.discovered_vehicles.add(vehicle_id)
            
            rospy.loginfo(f"Added vehicle: {vehicle_id}")

    def odom_callback(self, msg, vehicle_id):
        """Handle incoming odometry messages for any vehicle"""
        with self.lock:
            # 위치 정보 저장
            self.current_poses[vehicle_id] = [
                msg.pose.pose.position.x, 
                msg.pose.pose.position.y
            ]
            
            # 속도 정보 저장
            self.current_velocities[vehicle_id] = [
                msg.twist.twist.linear.x,   # 전진 속도
                msg.twist.twist.linear.y,   # 측면 속도
                msg.twist.twist.angular.z   # 각속도
            ]

    def publish_path(self, waypoints, vehicle_id):
        """Publish path for specific vehicle"""
        with self.lock:
            if vehicle_id not in self.path_pubs:
                return
                
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = rospy.Time.now()
            
            for wp in waypoints:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = wp[0]
                pose.pose.position.y = wp[1]
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            
            self.path_pubs[vehicle_id].publish(path_msg)

    def get_current_pose(self, vehicle_id):
        """특정 vehicle의 현재 pose 반환"""
        with self.lock:
            return self.current_poses.get(vehicle_id, None)

    def get_current_velocity(self, vehicle_id):
        """특정 vehicle의 현재 속도 반환 [vx, vy, angular_z]"""
        with self.lock:
            return self.current_velocities.get(vehicle_id, None)
    
    def get_vehicle_speed(self, vehicle_id):
        """특정 vehicle의 속도 크기 반환 (m/s)"""
        velocity = self.get_current_velocity(vehicle_id)
        if velocity is None:
            return 0.0
        
        # 속도 벡터의 크기 계산
        import math
        return math.sqrt(velocity[0]**2 + velocity[1]**2)
    
    def get_vehicle_speed_kmh(self, vehicle_id):
        """특정 vehicle의 속도 크기 반환 (km/h)"""
        speed_ms = self.get_vehicle_speed(vehicle_id)
        return speed_ms * 3.6
    
    def is_vehicle_moving(self, vehicle_id, threshold=0.1):
        """특정 vehicle이 움직이고 있는지 확인 (threshold: m/s)"""
        speed = self.get_vehicle_speed(vehicle_id)
        return speed > threshold

    def get_all_vehicles(self):
        """현재 발견된 모든 vehicle 목록 반환"""
        with self.lock:
            return list(self.discovered_vehicles)
    
    def get_all_poses(self):
        """모든 vehicle의 현재 pose 반환"""
        with self.lock:
            return dict(self.current_poses)
    
    def get_all_velocities(self):
        """모든 vehicle의 현재 속도 반환"""
        with self.lock:
            return dict(self.current_velocities)
    
    def print_vehicle_status(self):
        """모든 vehicle의 상태 정보 출력 (디버깅용)"""
        with self.lock:
            print(f"\n=== Vehicle Status ({len(self.discovered_vehicles)} vehicles) ===")
            
            for vehicle_id in sorted(self.discovered_vehicles):
                pose = self.current_poses.get(vehicle_id, [0, 0])
                velocity = self.current_velocities.get(vehicle_id, [0, 0, 0])
                speed = self.get_vehicle_speed(vehicle_id)
                
                print(f"{vehicle_id.upper()}: "
                      f"pos=({pose[0]:.2f}, {pose[1]:.2f}), "
                      f"speed={speed:.2f}m/s ({speed*3.6:.1f}km/h), "
                      f"vel=({velocity[0]:.2f}, {velocity[1]:.2f}), "
                      f"angular={velocity[2]:.3f}rad/s")
    
    def get_vehicle_info(self, vehicle_id):
        """특정 vehicle의 완전한 정보 반환"""
        with self.lock:
            if vehicle_id not in self.discovered_vehicles:
                return None
            
            pose = self.current_poses.get(vehicle_id, [0, 0])
            velocity = self.current_velocities.get(vehicle_id, [0, 0, 0])
            speed = self.get_vehicle_speed(vehicle_id)
            
            return {
                'vehicle_id': vehicle_id,
                'position': {'x': pose[0], 'y': pose[1]},
                'velocity': {
                    'linear_x': velocity[0],
                    'linear_y': velocity[1], 
                    'angular_z': velocity[2]
                },
                'speed_ms': speed,
                'speed_kmh': speed * 3.6,
                'is_moving': speed > 0.1
            }
    
    def remove_vehicle(self, vehicle_id):
        """특정 vehicle 제거 (subscriber, publisher 정리)"""
        with self.lock:
            if vehicle_id not in self.discovered_vehicles:
                return False
            
            # Subscriber 정리
            if vehicle_id in self.subscribers:
                self.subscribers[vehicle_id].unregister()
                del self.subscribers[vehicle_id]
            
            # Publisher 정리
            if vehicle_id in self.path_pubs:
                self.path_pubs[vehicle_id].unregister()
                del self.path_pubs[vehicle_id]
            
            # 데이터 정리
            self.current_poses.pop(vehicle_id, None)
            self.current_velocities.pop(vehicle_id, None)
            self.discovered_vehicles.discard(vehicle_id)
            
            rospy.loginfo(f"Removed vehicle: {vehicle_id}")
            return True
    
    def cleanup(self):
        """정리 작업"""
        if hasattr(self, 'discovery_timer'):
            self.discovery_timer.shutdown()
        
        with self.lock:
            # 모든 subscriber 정리
            for subscriber in self.subscribers.values():
                subscriber.unregister()
            
            # 모든 publisher 정리
            for publisher in self.path_pubs.values():
                publisher.unregister()
            
            self.subscribers.clear()
            self.path_pubs.clear()
            self.current_poses.clear()
            self.current_velocities.clear()
            self.discovered_vehicles.clear()
        
        rospy.loginfo("ROSManager cleanup completed")