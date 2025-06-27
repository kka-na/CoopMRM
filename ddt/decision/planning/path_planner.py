#!/usr/bin/env python3
import rospy
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import planning.libs.planning_helper as phelper

class PathPlanner:
    def __init__(self, map):
        self.map = map
        self.set_values()
    
    def set_values(self):
        phelper.lanelets = self.map.lanelets
        phelper.tiles = self.map.tiles
        phelper.tile_size = self.map.tile_size
        
        # Path length in meters (waypoints are 1m apart)
        self.path_length = 100  # 100m ahead
        
        # Current position
        self.current_pose = None
    
    def update_value(self, _current_pose):
        self.current_pose = _current_pose
        
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

    def execute(self):
        path = self.generate_straight_path()
        return path