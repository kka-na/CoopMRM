#!/usr/bin/env python3

import rospy
from coopmrm.msg import CoopData, TopologyInfo
import threading
import math
from topology_visualizer import TopologyVisualizer

class ROSManager:
    def __init__(self, my_namespace='/ego', num_targets=5):
        self.my_namespace = my_namespace
        self.my_vehicle_id = my_namespace.strip('/')
        self.num_targets = num_targets
        self.vehicle_data = {}  # {vehicle_id: CoopData}
        self.data_lock = threading.Lock()
        
        # Visualization
        self.visualizer = TopologyVisualizer(self.my_vehicle_id)
        
        # Publisher for topology info (NTC → V2X node) - 네임스페이스별
        topology_topic = f"{my_namespace}/topology_info"
        self.topology_info_pub = rospy.Publisher(topology_topic, TopologyInfo, queue_size=10)
        
        # Subscribe to all vehicle topics
        self.subscribers = []
        self._setup_subscribers()
        
    
    def _setup_subscribers(self):
        """모든 차량의 coopdata 구독"""
        vehicle_namespaces = ['/ego'] + [f'/target{i}' for i in range(1, self.num_targets + 1)]
        
        for namespace in vehicle_namespaces:
            topic_name = f"{namespace}/coopdata"
            sub = rospy.Subscriber(topic_name, CoopData, 
                                 lambda msg, ns=namespace: self._coopdata_callback(msg, ns),
                                 queue_size=10)
            self.subscribers.append(sub)
            rospy.logdebug(f"{self.my_vehicle_id} subscribed to {topic_name}")
    
    def _coopdata_callback(self, msg, namespace):
        """CoopData 메시지 콜백 - 이제 단순하게 모든 데이터 수신"""
        vehicle_id = msg.vehicle_id if msg.vehicle_id else namespace.strip('/')
        
        with self.data_lock:
            self.vehicle_data[vehicle_id] = msg
            rospy.logdebug(f"{self.my_vehicle_id}: Received coopdata from {vehicle_id}")
    
    def get_all_vehicles(self):
        """모든 차량 데이터 반환"""
        with self.data_lock:
            return dict(self.vehicle_data)
    
    def get_vehicle(self, vehicle_id):
        """특정 차량 데이터 반환"""
        with self.data_lock:
            return self.vehicle_data.get(vehicle_id, None)
    
    def am_i_fallback(self):
        """내가 fallback 차량인지 확인"""
        with self.data_lock:
            my_data = self.vehicle_data.get(self.my_vehicle_id)
            return my_data.is_fallback if my_data else False
    
    def get_fallback_vehicle(self):
        """fallback 차량 찾기"""
        with self.data_lock:
            for vehicle_id, data in self.vehicle_data.items():
                if hasattr(data, 'is_fallback') and data.is_fallback:
                    return vehicle_id, data
        return None, None
    
    def calculate_distance(self, pos1, pos2):
        """두 위치 간 거리 계산"""
        dx = pos1.position.x - pos2.position.x
        dy = pos1.position.y - pos2.position.y
        dz = pos1.position.z - pos2.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    def find_neighbors_in_range(self, vehicle_id, max_range=50.0):
        """통신 범위 내 이웃 차량 찾기"""
        neighbors = []
        
        with self.data_lock:
            rospy.loginfo(f"{self.my_vehicle_id}: Looking for neighbors of {vehicle_id}")
            rospy.loginfo(f"Available vehicles: {list(self.vehicle_data.keys())}")
            
            if vehicle_id not in self.vehicle_data:
                rospy.logwarn(f"{vehicle_id} not found in vehicle_data")
                return neighbors
            
            vehicle_data = self.vehicle_data[vehicle_id]
            if not hasattr(vehicle_data, 'pose') or vehicle_data.pose is None:
                rospy.logwarn(f"{vehicle_id} has no pose data")
                return neighbors
            
            vehicle_pos = vehicle_data.pose
            rospy.loginfo(f"{vehicle_id} position: ({vehicle_pos.position.x:.1f}, {vehicle_pos.position.y:.1f}, {vehicle_pos.position.z:.1f})")
            
            for other_id, other_data in self.vehicle_data.items():
                if other_id == vehicle_id:
                    continue
                
                if not hasattr(other_data, 'pose') or other_data.pose is None:
                    rospy.logwarn(f"{other_id} has no pose data, skipping")
                    continue
                
                other_pos = other_data.pose
                rospy.loginfo(f"{other_id} position: ({other_pos.position.x:.1f}, {other_pos.position.y:.1f}, {other_pos.position.z:.1f})")
                
                # 거리 계산 상세 로그
                dx = vehicle_pos.position.x - other_pos.position.x
                dy = vehicle_pos.position.y - other_pos.position.y
                dz = vehicle_pos.position.z - other_pos.position.z
                distance = math.sqrt(dx**2 + dy**2 + dz**2)
                
                rospy.loginfo(f"Distance calculation: dx={dx:.1f}, dy={dy:.1f}, dz={dz:.1f} -> distance={distance:.1f}m")
                
                if distance <= max_range:
                    neighbors.append((other_id, other_data, distance))
                    rospy.loginfo(f"  -> {other_id} is within range ({distance:.1f}m)")
        
        # 거리순 정렬
        neighbors.sort(key=lambda x: x[2])
        rospy.loginfo(f"Found {len(neighbors)} neighbors in {max_range}m range")
        return neighbors
    
    def get_yaw_from_quaternion(self, quat):
        """Quaternion에서 yaw 각도 추출"""
        import tf.transformations
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return euler[2]  # yaw
    
    def publish_topology_info(self, generation, is_assigner, children, max_gen=None):
        """TopologyInfo 메시지 퍼블리시 - V2X 노드로 전달"""
        topology_msg = TopologyInfo()
        # TopologyInfo에 header가 없으면 제거
        # topology_msg.header.stamp = rospy.Time.now()
        topology_msg.vehicle_id = self.my_vehicle_id
        topology_msg.generation_level = generation
        topology_msg.is_assigner = is_assigner
        topology_msg.my_children = children
        topology_msg.max_generation = max_gen if max_gen is not None else 0
        
        self.topology_info_pub.publish(topology_msg)
        
        rospy.loginfo(f"{self.my_vehicle_id}: Published topology info - "
                     f"Gen:{generation}, Assigner:{is_assigner}, Children:{children}, MaxGen:{topology_msg.max_generation}")
    
    def update_my_topology_info(self, generation, is_assigner, children, max_gen=None):
        """토폴로지 정보 업데이트 - TopologyInfo로 퍼블리시"""
        self.publish_topology_info(generation, is_assigner, children, max_gen)
    
    def publish_topology_visualization(self, generation_map, vehicle_info):
        """토폴로지 시각화 마커 퍼블리시"""
        all_vehicles = self.get_all_vehicles()
        if all_vehicles and generation_map and vehicle_info:
            rospy.logdebug(f"Generation map: {generation_map}")
            rospy.logdebug(f"Vehicle info keys: {list(vehicle_info.keys())}")
            
            # 각 차량의 generation 정보 로그
            for vehicle_id, info in vehicle_info.items():
                rospy.logdebug(f"  {vehicle_id}: generation={info.get('generation', 'unknown')}")
            
            self.visualizer.publish_topology_markers(all_vehicles, generation_map, vehicle_info)
        else:
            rospy.logwarn(f"{self.my_vehicle_id}: Cannot publish visualization - "
                         f"vehicles:{len(all_vehicles) if all_vehicles else 0}, "
                         f"gen_map:{bool(generation_map)}, vehicle_info:{bool(vehicle_info)}")
    
    def clear_visualization(self):
        """시각화 마커 제거"""
        self.visualizer.clear_markers()