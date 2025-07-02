#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import threading

class TopologyVisualizer:
    def __init__(self, my_vehicle_id):
        self.my_vehicle_id = my_vehicle_id
        self.publisher = rospy.Publisher('/topology_markers', MarkerArray, queue_size=10)
        
        # Generation별 색상 정의 (255는 제거 - 마커 표시하지 않음)
        self.generation_colors = {
            0: {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 0.8},  # Red - Fallback
            1: {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 0.8},  # Green
            2: {'r': 0.0, 'g': 0.0, 'b': 1.0, 'a': 0.8},  # Blue
            3: {'r': 1.0, 'g': 1.0, 'b': 0.0, 'a': 0.8},  # Yellow
            4: {'r': 1.0, 'g': 0.0, 'b': 1.0, 'a': 0.8},  # Magenta
            5: {'r': 0.0, 'g': 1.0, 'b': 1.0, 'a': 0.8},  # Cyan
        }
        
        self.lock = threading.Lock()
    
    def publish_topology_markers(self, all_vehicles_data, generation_map, vehicle_info):
        """토폴로지 마커 퍼블리시"""
        marker_array = MarkerArray()
        marker_id = 0
        
        with self.lock:
            # 1. 차량 위치 마커들
            for vehicle_id, vehicle_data in all_vehicles_data.items():
                if hasattr(vehicle_data, 'pose') and vehicle_data.pose:
                    # vehicle_info에서 generation 정보 가져오기 (더 정확함)
                    generation = 0  # 기본값
                    if vehicle_id in vehicle_info:
                        generation = vehicle_info[vehicle_id].get('generation', 0)
                    else:
                        # fallback으로 vehicle_data에서 가져오기
                        generation = getattr(vehicle_data, 'generation_level', 0)
                    
                    # Generation 255인 경우 마커 생성하지 않음
                    if generation == 255:
                        rospy.logdebug(f"Skipping markers for {vehicle_id}: generation=255 (unassigned)")
                        continue
                    
                    rospy.logdebug(f"Vehicle {vehicle_id}: generation={generation}")
                    
                    # 차량 마커 생성
                    vehicle_marker = self._create_vehicle_marker(
                        vehicle_id, vehicle_data.pose, generation, marker_id
                    )
                    marker_array.markers.append(vehicle_marker)
                    marker_id += 1
                    
                    # 텍스트 라벨 마커 생성
                    text_marker = self._create_text_marker(
                        vehicle_id, vehicle_data.pose, generation, marker_id
                    )
                    marker_array.markers.append(text_marker)
                    marker_id += 1
            
            # 2. 연결선 마커들 (부모-자식 관계)
            for vehicle_id, info in vehicle_info.items():
                if 'children' in info and info['children']:
                    parent_data = all_vehicles_data.get(vehicle_id)
                    if not parent_data or not hasattr(parent_data, 'pose'):
                        continue
                    
                    parent_generation = info.get('generation', 0)
                    
                    # 부모가 generation 255면 연결선도 그리지 않음
                    if parent_generation == 255:
                        rospy.logdebug(f"Skipping connections for {vehicle_id}: parent generation=255")
                        continue
                    
                    for child_id in info['children']:
                        child_data = all_vehicles_data.get(child_id)
                        child_info = vehicle_info.get(child_id, {})
                        child_generation = child_info.get('generation', 0)
                        
                        # 자식이 generation 255면 해당 연결선 그리지 않음
                        if child_generation == 255:
                            rospy.logdebug(f"Skipping connection to {child_id}: child generation=255")
                            continue
                        
                        if child_data and hasattr(child_data, 'pose'):
                            connection_marker = self._create_connection_marker(
                                parent_data.pose, child_data.pose, 
                                parent_generation, marker_id
                            )
                            marker_array.markers.append(connection_marker)
                            marker_id += 1
        
        # 퍼블리시
        if marker_array.markers:
            self.publisher.publish(marker_array)
            
            # 생성된 마커들의 generation 정보 로그
            vehicle_markers = [m for m in marker_array.markers if m.ns == "vehicles"]
            for marker in vehicle_markers:
                # 마커 ID로부터 vehicle_id 추정 (디버깅용)
                rospy.logdebug(f"Marker ID {marker.id}: color=({marker.color.r:.1f},{marker.color.g:.1f},{marker.color.b:.1f})")
        
    
    def _create_vehicle_marker(self, vehicle_id, pose, generation, marker_id):
        """차량 위치 마커 생성"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "vehicles"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # 위치
        marker.pose = pose
        
        # 크기 수정 - 훨씬 작게
        base_size = 3.0  # 3.0에서 1.0으로 축소
        size_multiplier = 1.0 + (generation * 0.1)  # 0.1에서 0.05로 축소
        marker.scale.x = base_size * size_multiplier
        marker.scale.y = base_size * size_multiplier
        marker.scale.z = base_size * size_multiplier
        
        # 색상 (generation별) - 명시적으로 ColorRGBA 사용
        color = self.generation_colors.get(generation, self.generation_colors[0])
        marker.color = ColorRGBA()
        marker.color.r = float(color['r'])
        marker.color.g = float(color['g'])
        marker.color.b = float(color['b'])
        marker.color.a = float(color['a'])
        
        # Lifetime
        marker.lifetime = rospy.Duration(0)
        
        rospy.logdebug(f"Created vehicle marker for {vehicle_id} (Gen {generation}): "
                      f"color=({marker.color.r:.1f},{marker.color.g:.1f},{marker.color.b:.1f})")
        
        return marker
    
    def _create_text_marker(self, vehicle_id, pose, generation, marker_id):
        """차량 ID 텍스트 마커 생성"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "vehicle_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # 위치 (차량 위에)
        marker.pose = pose
        marker.pose.position.z += 2.0  # 4.0에서 2.0으로 낮춤
        
        # 텍스트 - generation 255는 이미 필터링되어 여기 도달하지 않음
        marker.text = f"{vehicle_id}\n[Gen {generation}]"
        
        # 크기 축소
        marker.scale.z = 1.0  # 2.0에서 1.0으로 축소
        
        # 색상 (흰색, 검은 배경으로 잘 보이게)
        marker.color = ColorRGBA()
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        # Lifetime
        marker.lifetime = rospy.Duration(0)
        
        return marker
    
    def _create_connection_marker(self, parent_pose, child_pose, generation, marker_id):
        """부모-자식 연결선 마커 생성"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "connections"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # 연결선 점들
        start_point = Point()
        start_point.x = parent_pose.position.x
        start_point.y = parent_pose.position.y
        start_point.z = parent_pose.position.z + 1.0  # 약간 위로 올려서 바닥에서 분리
        
        end_point = Point()
        end_point.x = child_pose.position.x
        end_point.y = child_pose.position.y
        end_point.z = child_pose.position.z + 1.0  # 약간 위로 올려서 바닥에서 분리
        
        marker.points = [start_point, end_point]
        
        # 선 두께
        marker.scale.x = 0.8  # 조금 더 두껍게 해서 잘 보이게
        
        # 색상 (부모 generation 색상, 조금 더 투명하게)
        color = self.generation_colors.get(generation, self.generation_colors[0])
        marker.color = ColorRGBA()
        marker.color.r = float(color['r'])
        marker.color.g = float(color['g'])
        marker.color.b = float(color['b'])
        marker.color.a = 0.7  # 연결선은 조금 더 투명
        
        # Lifetime
        marker.lifetime = rospy.Duration(0)
        
        return marker
    
    def clear_markers(self):
        """모든 마커 제거"""
        marker_array = MarkerArray()
        
        # 각 namespace별로 삭제 마커 생성
        namespaces = ["vehicles", "vehicle_labels", "connections"]
        
        for ns in namespaces:
            clear_marker = Marker()
            clear_marker.header.frame_id = "map"
            clear_marker.header.stamp = rospy.Time.now()
            clear_marker.ns = ns
            clear_marker.action = Marker.DELETEALL
            marker_array.markers.append(clear_marker)
        
        self.publisher.publish(marker_array)
    
    def get_generation_color_info(self):
        """Generation별 색상 정보 반환 (디버깅용)"""
        return self.generation_colors