#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from localization.hdmap.map import MAP
import planning.libs.planning_helper as phelper

class StraightPathTest:
    def __init__(self, map_name):
        rospy.init_node('straight_path_test')
        
        # Initialize map
        self.map = MAP(map_name)
        phelper.lanelets = self.map.lanelets
        phelper.tiles = self.map.tiles
        phelper.tile_size = self.map.tile_size
        
        # Path length in meters (waypoints are 1m apart)
        self.path_length = 100  # 100m ahead
        
        # Current position
        self.current_pose = None
        
        # Publishers
        self.path_pub = rospy.Publisher('/ego/straight_path', Path, queue_size=1)
        # Subscriber
        rospy.Subscriber('/ego/pose', PoseStamped, self.pose_callback)
        
        rospy.loginfo("Straight path test node initialized")

    def pose_callback(self, msg):
        """Handle incoming pose messages"""
        self.current_pose = [msg.pose.position.x, msg.pose.position.y]
        
        # Generate and publish path
        path = self.generate_straight_path()
        if path:
            self.publish_path(path)

    def generate_straight_path(self):
        """Generate straight path from current position"""
        if self.current_pose is None:
            return None
        
        # Match current position to lanelet
        idnidx = phelper.lanelet_matching(self.current_pose)
        if idnidx is None:
            rospy.logwarn("Could not match position to lanelet")
            return None
        
        # Get straight path
        path_waypoints, _ = phelper.get_straight_path(idnidx, self.path_length)
        
        # Smooth the path
        if len(path_waypoints) > 2:
            smoothed_path = phelper.smooth_interpolate(path_waypoints)
            return smoothed_path
        
        return path_waypoints

    def publish_path(self, waypoints):
        """Publish path for visualization in RViz"""
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
        
        self.path_pub.publish(path_msg)
        rospy.loginfo(f"Published path with {len(waypoints)} waypoints")

def main():
    # Map name - change this to your map
    map_name = 'Midan'  # or whatever map you're using
    
    try:
        test_node = StraightPathTest(map_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()