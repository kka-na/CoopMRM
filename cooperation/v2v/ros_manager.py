#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from coopmrm.msg import CoopData
import threading

class ROSManager:
    def __init__(self):
        # 각 차량별 데이터 저장
        self.vehicle_data = {}  # {vehicle_id: {'pose': PoseStamped, 'path': Path}}
        self.publishers = {}    # {vehicle_id: Publisher}
        self.subscribers = {}   # {topic_name: Subscriber}
        
        # 토픽 모니터링을 위한 스레드
        self.topic_monitor_thread = threading.Thread(target=self.monitor_topics)
        self.topic_monitor_thread.daemon = True
        self.topic_monitor_thread.start()
        
        # 기본 ego 차량 설정
        self.setup_vehicle_topics('ego')
        
        rospy.loginfo("ROS Manager initialized with dynamic topic handling")
    
    def monitor_topics(self):
        """동적으로 새로운 토픽을 감지하고 구독"""
        rate = rospy.Rate(1)  # 1Hz로 토픽 체크
        
        while not rospy.is_shutdown():
            try:
                # 현재 활성 토픽 목록 가져오기
                topic_list = rospy.get_published_topics()
                
                # target 차량들 찾기
                for topic_name, topic_type in topic_list:
                    if '/pose' in topic_name and topic_type == 'geometry_msgs/PoseStamped':
                        vehicle_id = self.extract_vehicle_id(topic_name)
                        if vehicle_id and vehicle_id not in self.vehicle_data:
                            self.setup_vehicle_topics(vehicle_id)
                            rospy.loginfo(f"Added new vehicle: {vehicle_id}")
                
                rate.sleep()
            except Exception as e:
                rospy.logwarn(f"Topic monitoring error: {e}")
                rate.sleep()
    
    def extract_vehicle_id(self, topic_name):
        """토픽 이름에서 차량 ID 추출"""
        # /ego/pose -> ego, /target1/pose -> target1
        parts = topic_name.split('/')
        if len(parts) >= 2:
            vehicle_id = parts[1]
            if vehicle_id in ['ego'] or vehicle_id.startswith('target'):
                return vehicle_id
        return None
    
    def setup_vehicle_topics(self, vehicle_id):
        """특정 차량의 토픽 설정"""
        if vehicle_id in self.vehicle_data:
            return
        
        # 데이터 저장소 초기화
        self.vehicle_data[vehicle_id] = {'pose': None, 'path': None}
        
        # Publisher 생성
        pub_topic = f'/{vehicle_id}/coopdata'
        self.publishers[vehicle_id] = rospy.Publisher(pub_topic, CoopData, queue_size=10)
        
        # Subscribers 생성
        pose_topic = f'/{vehicle_id}/pose'
        path_topic = f'/{vehicle_id}/path'
        
        pose_sub = rospy.Subscriber(
            pose_topic, 
            PoseStamped, 
            lambda msg, vid=vehicle_id: self.pose_callback(msg, vid)
        )
        
        path_sub = rospy.Subscriber(
            path_topic, 
            Path, 
            lambda msg, vid=vehicle_id: self.path_callback(msg, vid)
        )
        
        self.subscribers[pose_topic] = pose_sub
        self.subscribers[path_topic] = path_sub
        
        rospy.loginfo(f"Setup topics for vehicle: {vehicle_id}")
    
    def pose_callback(self, msg, vehicle_id):
        """Pose 콜백 (차량별)"""
        if vehicle_id in self.vehicle_data:
            self.vehicle_data[vehicle_id]['pose'] = msg
    
    def path_callback(self, msg, vehicle_id):
        """Path 콜백 (차량별)"""
        if vehicle_id in self.vehicle_data:
            self.vehicle_data[vehicle_id]['path'] = msg
    
    def get_latest_pose(self, vehicle_id):
        """특정 차량의 최신 pose 반환"""
        if vehicle_id in self.vehicle_data:
            return self.vehicle_data[vehicle_id]['pose']
        return None
    
    def get_latest_path(self, vehicle_id):
        """특정 차량의 최신 path 반환"""
        if vehicle_id in self.vehicle_data:
            return self.vehicle_data[vehicle_id]['path']
        return None
    
    def has_pose_data(self, vehicle_id):
        """특정 차량의 pose 데이터 존재 여부"""
        return (vehicle_id in self.vehicle_data and 
                self.vehicle_data[vehicle_id]['pose'] is not None)
    
    def has_path_data(self, vehicle_id):
        """특정 차량의 path 데이터 존재 여부"""
        return (vehicle_id in self.vehicle_data and 
                self.vehicle_data[vehicle_id]['path'] is not None)
    
    def publish_coop_data(self, coop_msg, vehicle_id):
        """특정 차량의 CoopData 발행"""
        if vehicle_id in self.publishers:
            self.publishers[vehicle_id].publish(coop_msg)
            rospy.logdebug(f"Published CoopData for vehicle: {vehicle_id}")
    
    def get_active_vehicles(self):
        """현재 활성 차량 목록 반환"""
        return list(self.vehicle_data.keys())
    
    def get_vehicles_with_data(self):
        """데이터가 있는 차량들만 반환"""
        vehicles_with_data = []
        for vehicle_id in self.vehicle_data:
            if self.has_pose_data(vehicle_id):
                vehicles_with_data.append(vehicle_id)
        return vehicles_with_data