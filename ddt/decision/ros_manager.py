#!/usr/bin/env python
import rospy
import threading
from pyproj import Proj, Transformer

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2

class ROSManager:
    def __init__(self, type, map):
        self.type = type
        self.map = map
        self.discovered_vehicles = set()  # 발견된 vehicle들
        self.lock = threading.Lock()  # thread-safe를 위한 lock
        self.set_values()
        self.set_protocol()
        
        # 주기적으로 새로운 pose 토픽을 찾는 타이머
        self.discovery_timer = rospy.Timer(rospy.Duration(2.0), self.discover_new_vehicles)
    
    def set_values(self):
        proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=self.map.base_lla[0], lon_0=self.map.base_lla[1], h_0=self.map.base_lla[2])
        self.geo2enu_transformer = Transformer.from_proj(proj_wgs84, proj_enu)
        self.enu2geo_transformter = Transformer.from_proj(proj_enu, proj_wgs84)

        self.current_poses = {}  # {vehicle_id: [x, y]}
        self.subscribers = {}    # {vehicle_id: subscriber}
        self.path_pubs = {}      # {vehicle_id: publisher}

    def set_protocol(self):
        # map publish
        pointcloud_pub = rospy.Publisher('/map/combined', PointCloud2, queue_size=1, latch=True)
        pointcloud_pub.publish(self.map.lmap_viz)

        # ego는 기본으로 추가
        self.add_vehicle('ego')

    def discover_new_vehicles(self, event):
        """주기적으로 새로운 pose 토픽을 찾아서 추가"""
        try:
            # 현재 발행되고 있는 모든 토픽 목록 가져오기
            topics = rospy.get_published_topics()
            
            for topic_name, topic_type in topics:
                # /xxx/pose 패턴의 토픽 찾기
                if topic_name.endswith('/pose') and topic_type == 'geometry_msgs/PoseStamped':
                    # vehicle_id 추출 (예: /target1/pose -> target1)
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
                
            # Subscriber 추가
            topic_name = f'/{vehicle_id}/pose'
            self.subscribers[vehicle_id] = rospy.Subscriber(
                topic_name, 
                PoseStamped, 
                lambda msg, vid=vehicle_id: self.pose_callback(msg, vid)
            )
            
            # Publisher 추가
            path_topic = f'/{vehicle_id}/path'
            self.path_pubs[vehicle_id] = rospy.Publisher(path_topic, Path, queue_size=1)
            
            # 발견된 vehicle 목록에 추가
            self.discovered_vehicles.add(vehicle_id)
            
            rospy.loginfo(f"Added vehicle: {vehicle_id}")

    def pose_callback(self, msg, vehicle_id):
        """Handle incoming pose messages for any vehicle"""
        with self.lock:
            self.current_poses[vehicle_id] = [msg.pose.position.x, msg.pose.position.y]

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

    def get_all_vehicles(self):
        """현재 발견된 모든 vehicle 목록 반환"""
        with self.lock:
            return list(self.discovered_vehicles)