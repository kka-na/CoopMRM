#!/usr/bin/env python
import rospy
from pyproj import Proj, Transformer

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2

class ROSManager:
    def __init__(self, type,map):
        self.type = type
        self.map = map
        self.set_values()
        self.set_protocol()
    
    def set_values(self):
        proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=self.map.base_lla[0], lon_0=self.map.base_lla[1], h_0=self.map.base_lla[2])
        self.geo2enu_transformer = Transformer.from_proj(proj_wgs84, proj_enu)
        self.enu2geo_transformter = Transformer.from_proj(proj_enu, proj_wgs84)

        self.current_pose = None

    def set_protocol(self):
        # map publish
        pointcloud_pub = rospy.Publisher('/map/combined', PointCloud2, queue_size=1, latch=True)
        pointcloud_pub.publish(self.map.lmap_viz)

        rospy.Subscriber('/ego/pose', PoseStamped, self.pose_callback)
        self.path_pub = rospy.Publisher('/ego/path', Path, queue_size=1)
    
    def pose_callback(self, msg):
        """Handle incoming pose messages"""
        self.current_pose = [msg.pose.position.x, msg.pose.position.y]

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
