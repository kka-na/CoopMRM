#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from coopmrm.msg import CoopData  # 실제 패키지명으로 변경 필요

class CoopDataPackager:
    def __init__(self, ros_manager):
        # Vehicle configuration
        self.vehicle_id = rospy.get_param('~vehicle_id', 'vehicle_001')
        self.generation_level = rospy.get_param('~generation_level', 1)
        self.trust_score = rospy.get_param('~trust_score', 95)
        
        # Default phase and message type
        self.current_phase = 0  # PROPAGATE
        self.current_message_type = 2  # UPDATE

        self.RM = ros_manager
            
    def execute(self):
            
        # 데이터가 있는 모든 차량에 대해 처리
        vehicles_with_data = self.RM.get_vehicles_with_data()
        
        for vehicle_id in vehicles_with_data:
            coop_msg = self.create_coop_data_message(vehicle_id)
            if coop_msg:
                self.RM.publish_coop_data(coop_msg,vehicle_id)
    
    def create_coop_data_message(self, vehicle_id):
        """Create CoopData message from current vehicle state"""
        try:
            coop_msg = CoopData()
            
            # Header
            coop_msg.header.stamp = rospy.Time.now()
            coop_msg.header.frame_id = "map"
            
            # Vehicle Info
            coop_msg.vehicle_id = vehicle_id
            coop_msg.pose = self.RM.get_latest_pose(vehicle_id).pose
            coop_msg.generation_level = self.generation_level
            coop_msg.is_fallback = False
            
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
    
    def extract_waypoints(self, vehicle_id):
        """Extract waypoints from path data"""
        waypoints = []
        
        if self.RM.has_path_data(vehicle_id):
            path = self.RM.get_latest_path(vehicle_id)
            
            for pose_stamped in path.poses:
                waypoint = Point()
                waypoint.x = pose_stamped.pose.position.x
                waypoint.y = pose_stamped.pose.position.y
                waypoint.z = pose_stamped.pose.position.z
                waypoints.append(waypoint)
        
        return waypoints
    
    def set_phase(self, phase):
        """Set current cooperation phase"""
        self.current_phase = phase
    
    def set_message_type(self, msg_type):
        """Set current message type"""
        self.current_message_type = msg_type
    
    def set_fallback_mode(self, is_fallback):
        """Set fallback mode status"""
        self.is_fallback = is_fallback