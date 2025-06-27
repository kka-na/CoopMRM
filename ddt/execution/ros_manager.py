#!/usr/bin/env python
import rospy
import copy
import tf


from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

CAR_TEMPLATE = {'state': 0, 'x': 0, 'y': 0, 't': 0, 'v': 0, 'path': []}

class ROSManager:
    def __init__(self, type, targets_num):
        self.type = type
        self.targets_num = targets_num
        self.set_values()
        self.set_protocol()
    
    def set_values(self):
        self.ego = copy.deepcopy(CAR_TEMPLATE)
        self.targets = []
        for i in range(self.targets_num):
            self.targets.append(copy.deepcopy(CAR_TEMPLATE))

        self.target_velocity = 30

    def set_protocol(self):
        rospy.Subscriber('/ego/pose', PoseStamped, 
                     lambda msg: self.pose_callback(msg, self.ego))
        rospy.Subscriber('/ego/path', Path, 
                        lambda msg: self.path_callback(msg, self.ego))
        
        # targets 구독
        for i in range(self.targets_num):
            rospy.Subscriber(f'/target{i+1}/pose', PoseStamped, 
                            lambda msg, idx=i: self.pose_callback(msg, self.targets[idx]))
            rospy.Subscriber(f'/target{i+1}/path', Path, 
                            lambda msg, idx=i: self.path_callback(msg, self.targets[idx]))
            
        self.lh_test_pub = rospy.Publisher(f'{self.type}/look_a_head', Marker, queue_size=1)

    def pose_callback(self, msg, car_dict):
        """공통 pose 콜백 - car_dict에 따라 다른 객체 업데이트"""
        car_dict['x'] = msg.pose.position.x
        car_dict['y'] = msg.pose.position.y
        quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        car_dict['t'] = yaw

    def path_callback(self, msg, car_dict):
        """공통 path 콜백"""
        car_dict['path'] = [(p.pose.position.x, p.pose.position.y) 
                            for p in msg.poses]

    def pub_lh(self, lh):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.header.frame_id = 'world'
        marker.ns = 'lookahead'
        marker.id = 1
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 0.7
        marker.scale.y = 0.7
        marker.scale.z = 0.7
        if self.type == 'ego':
            marker.color.r = 241/255
            marker.color.g = 76/255
            marker.color.b = 152/255
        else:
            marker.color.r = 94/255
            marker.color.g = 204/255
            marker.color.b = 243/255
        marker.color.a = 1
        marker.pose.position.x = lh[0]
        marker.pose.position.y = lh[1]
        marker.pose.position.z = 1.0
        self.lh_test_pub.publish(marker)

