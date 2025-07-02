#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Pose
from coopmrm.msg import CoopData  # 실제 패키지명으로 변경 필요

class CoopDataPackager:
    def __init__(self, ros_manager):
        # Vehicle configuration
        self.vehicle_id = rospy.get_param('~vehicle_id', 'vehicle_001')
        self.generation_level = rospy.get_param('~generation_level', 255)
        self.trust_score = rospy.get_param('~trust_score', 95)
        
        # Default phase and message type
        self.current_phase = 0  # PROPAGATE
        self.current_message_type = 2  # UPDATE

        self.RM = ros_manager
            
    def execute(self):
        """모든 차량에 대해 CoopData 메시지 생성 및 발행"""
        # 데이터가 있는 모든 차량에 대해 처리 (odom 기반으로 변경)
        vehicles_with_data = self.RM.get_vehicles_with_data()
        
        for vehicle_id in vehicles_with_data:
            coop_msg = self.create_coop_data_message(vehicle_id)
            if coop_msg:
                self.RM.publish_coop_data(coop_msg, vehicle_id)
    
    def create_coop_data_message(self, vehicle_id):
        """Create CoopData message from current vehicle state (Odometry 기반)"""
        try:
            coop_msg = CoopData()
            
            # Header
            coop_msg.header.stamp = rospy.Time.now()
            coop_msg.header.frame_id = "map"
            
            # Vehicle Info
            coop_msg.vehicle_id = vehicle_id
            
            # Pose 정보 (Odometry에서 추출)
            coop_msg.pose = self.extract_pose_from_odom(vehicle_id)
            if coop_msg.pose is None:
                rospy.logwarn(f"No pose data available for {vehicle_id}")
                return None
            
            # TopologyInfo에서 generation_level 가져오기
            topology = self.RM.get_topology_info(vehicle_id)
            if topology is not None:
                coop_msg.generation_level = topology['generation_level']
                # 추가 토폴로지 정보도 설정 (CoopData.msg에 해당 필드가 있다면)
                if hasattr(coop_msg, 'is_assigner'):
                    coop_msg.is_assigner = topology['is_assigner']
                if hasattr(coop_msg, 'my_children'):
                    coop_msg.my_children = topology['my_children']
                if hasattr(coop_msg, 'max_generation'):
                    coop_msg.max_generation = topology['max_generation']
                
                rospy.logdebug(f"Applied topology info to {vehicle_id}: gen={topology['generation_level']}")
            else:
                # TopologyInfo가 없으면 기본값 사용
                coop_msg.generation_level = self.generation_level
                rospy.logdebug(f"No topology info for {vehicle_id}, using default gen={self.generation_level}")
            
            # Fallback 상태
            coop_msg.is_fallback = self.extract_fallback(vehicle_id)
            
            # Path Info
            coop_msg.waypoints = self.extract_waypoints(vehicle_id)
            coop_msg.trust_score = self.trust_score
            
            # Control/Status
            coop_msg.phase = self.current_phase
            coop_msg.message_type = self.current_message_type
            
            return coop_msg
            
        except Exception as e:
            rospy.logerr(f"Failed to create CoopData message for {vehicle_id}: {e}")
            return None
    
    def extract_pose_from_odom(self, vehicle_id):
        """Odometry에서 Pose 정보 추출"""
        try:
            # Odometry 데이터에서 pose 추출
            if self.RM.has_odom_data(vehicle_id):
                odom = self.RM.get_latest_odom(vehicle_id)
                return odom.pose.pose
            
            # 호환성을 위해 직접 pose 데이터도 확인
            elif self.RM.has_pose_data(vehicle_id):
                return self.RM.get_latest_pose(vehicle_id)
            
            return None
            
        except Exception as e:
            rospy.logwarn(f"Failed to extract pose for {vehicle_id}: {e}")
            return None
    
    def extract_waypoints(self, vehicle_id):
        """Extract waypoints from path data"""
        waypoints = []
        
        try:
            if self.RM.has_path_data(vehicle_id):
                path = self.RM.get_latest_path(vehicle_id)
                
                for pose_stamped in path.poses:
                    waypoint = Point()
                    waypoint.x = pose_stamped.pose.position.x
                    waypoint.y = pose_stamped.pose.position.y
                    waypoint.z = pose_stamped.pose.position.z
                    waypoints.append(waypoint)
        except Exception as e:
            rospy.logwarn(f"Failed to extract waypoints for {vehicle_id}: {e}")
        
        return waypoints

    def extract_fallback(self, vehicle_id):
        """차량의 fallback 상태 추출"""
        fallback = False
        
        try:
            if self.RM.has_car_state_data(vehicle_id):
                car_state = self.RM.get_latest_car_state(vehicle_id)
                
                # car_state가 1보다 크면 fallback 상태로 판단
                # 0: STOP, 1: GO, 3: FALLBACK
                if car_state is not None and car_state >= 3:
                    fallback = True
        except Exception as e:
            rospy.logwarn(f"Failed to extract fallback state for {vehicle_id}: {e}")
            
        return fallback
    
    def get_vehicle_velocity_info(self, vehicle_id):
        """차량의 속도 정보 반환 (디버깅/로깅용)"""
        try:
            if self.RM.has_velocity_data(vehicle_id):
                velocity = self.RM.get_latest_velocity(vehicle_id)
                speed_ms = self.RM.get_vehicle_speed(vehicle_id)
                speed_kmh = self.RM.get_vehicle_speed_kmh(vehicle_id)
                is_moving = self.RM.is_vehicle_moving(vehicle_id)
                
                return {
                    'linear_x': velocity['linear_x'],
                    'linear_y': velocity['linear_y'], 
                    'angular_z': velocity['angular_z'],
                    'speed_ms': speed_ms,
                    'speed_kmh': speed_kmh,
                    'is_moving': is_moving
                }
        except Exception as e:
            rospy.logwarn(f"Failed to get velocity info for {vehicle_id}: {e}")
            
        return None
    
    def get_vehicle_position_info(self, vehicle_id):
        """차량의 위치 정보 반환 (디버깅/로깅용)"""
        try:
            position = self.RM.get_vehicle_position(vehicle_id)
            if position:
                return {'x': position[0], 'y': position[1]}
        except Exception as e:
            rospy.logwarn(f"Failed to get position info for {vehicle_id}: {e}")
            
        return None
    
    def create_enhanced_coop_data_message(self, vehicle_id):
        """속도 정보가 포함된 확장 CoopData 메시지 생성 (향후 확장용)"""
        try:
            # 기본 CoopData 메시지 생성
            coop_msg = self.create_coop_data_message(vehicle_id)
            if coop_msg is None:
                return None
            
            # 추가 정보 로깅 (CoopData 메시지에 직접 추가하지 않고 로깅용)
            velocity_info = self.get_vehicle_velocity_info(vehicle_id)
            position_info = self.get_vehicle_position_info(vehicle_id)
            
            if velocity_info and position_info:
                rospy.logdebug(f"Vehicle {vehicle_id}: "
                              f"pos=({position_info['x']:.2f}, {position_info['y']:.2f}), "
                              f"speed={velocity_info['speed_ms']:.2f}m/s, "
                              f"moving={velocity_info['is_moving']}")
            
            return coop_msg
            
        except Exception as e:
            rospy.logerr(f"Failed to create enhanced CoopData message for {vehicle_id}: {e}")
            return None
    
    def validate_vehicle_data(self, vehicle_id):
        """차량 데이터 유효성 검사"""
        validation_result = {
            'vehicle_id': vehicle_id,
            'has_odom': self.RM.has_odom_data(vehicle_id),
            'has_path': self.RM.has_path_data(vehicle_id),
            'has_car_state': self.RM.has_car_state_data(vehicle_id),
            'has_velocity': self.RM.has_velocity_data(vehicle_id),
            'has_topology': self.RM.has_topology_data(vehicle_id),
            'is_valid': False
        }
        
        # 최소한 odom 데이터가 있어야 유효
        validation_result['is_valid'] = validation_result['has_odom']
        
        return validation_result
    
    def execute_with_validation(self):
        """데이터 유효성 검사와 함께 CoopData 발행"""
        vehicles_with_data = self.RM.get_vehicles_with_data()
        
        valid_count = 0
        total_count = len(vehicles_with_data)
        
        for vehicle_id in vehicles_with_data:
            validation = self.validate_vehicle_data(vehicle_id)
            
            if validation['is_valid']:
                coop_msg = self.create_coop_data_message(vehicle_id)
                if coop_msg:
                    self.RM.publish_coop_data(coop_msg, vehicle_id)
                    valid_count += 1
            else:
                rospy.logwarn(f"Invalid data for vehicle {vehicle_id}: {validation}")
        
        if total_count > 0:
            rospy.logdebug(f"Published CoopData for {valid_count}/{total_count} vehicles")
    
    def set_phase(self, phase):
        """Set current cooperation phase"""
        self.current_phase = phase
        rospy.logdebug(f"CoopData phase set to: {phase}")
    
    def set_message_type(self, msg_type):
        """Set current message type"""
        self.current_message_type = msg_type
        rospy.logdebug(f"CoopData message type set to: {msg_type}")
    
    def set_trust_score(self, trust_score):
        """Set trust score for all vehicles"""
        if 0 <= trust_score <= 100:
            self.trust_score = trust_score
            rospy.logdebug(f"Trust score set to: {trust_score}")
        else:
            rospy.logwarn(f"Invalid trust score: {trust_score}. Must be 0-100.")
    
    def set_generation_level(self, generation_level):
        """Set generation level for all vehicles (fallback용)"""
        if generation_level >= 0:
            self.generation_level = generation_level
            rospy.logdebug(f"Default generation level set to: {generation_level}")
        else:
            rospy.logwarn(f"Invalid generation level: {generation_level}. Must be >= 0.")
    
    def print_status(self):
        """현재 CoopDataPackager 상태 출력"""
        vehicles = self.RM.get_vehicles_with_data()
        
        print(f"\n=== CoopDataPackager Status ===")
        print(f"Active vehicles: {len(vehicles)}")
        print(f"Default generation level: {self.generation_level}")
        print(f"Trust score: {self.trust_score}")
        print(f"Current phase: {self.current_phase}")
        print(f"Message type: {self.current_message_type}")
        
        for vehicle_id in vehicles:
            validation = self.validate_vehicle_data(vehicle_id)
            velocity_info = self.get_vehicle_velocity_info(vehicle_id)
            fallback_status = self.extract_fallback(vehicle_id)
            topology = self.RM.get_topology_info(vehicle_id)
            
            print(f"\n{vehicle_id.upper()}:")
            print(f"  - Valid data: {validation['is_valid']}")
            print(f"  - Has topology: {validation['has_topology']}")
            if topology:
                print(f"  - Generation level: {topology['generation_level']}")
                print(f"  - Is assigner: {topology['is_assigner']}")
                print(f"  - Children: {topology['my_children']}")
            else:
                print(f"  - Using default gen level: {self.generation_level}")
            print(f"  - Fallback: {fallback_status}")
            if velocity_info:
                print(f"  - Speed: {velocity_info['speed_ms']:.2f}m/s ({velocity_info['speed_kmh']:.1f}km/h)")
                print(f"  - Moving: {velocity_info['is_moving']}")
    
    def cleanup(self):
        """정리 작업"""
        rospy.loginfo("CoopDataPackager cleanup completed")