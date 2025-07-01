#!/usr/bin/env python3
"""
Waypoints + Boundary를 하나의 PointCloud2로 색상 구분 시각화
노란색: waypoints, 회색: boundary
"""

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct

class MapViz:
    """Waypoints와 Boundary를 하나의 PointCloud로 시각화"""
    
    def __init__(self):
        self.frame_id = 'map'
        
        
        # 색상 정의 (RGB)
        self.waypoint_color = [255, 255, 0]    # 노란색
        self.boundary_color = [128, 128, 128]  # 회색
        
    def create_combined_pointcloud(self, lanelets, waypoint_subsample=2, boundary_subsample=3):
        """
        Waypoints와 Boundary를 하나의 PointCloud2로 생성
        
        Args:
            lanelets: lanelet 데이터
            waypoint_subsample: waypoint 간격 (3 = 3개마다 1개)
            boundary_subsample: boundary 간격 (2 = 2개마다 1개)
        """
        points = []
        colors = []

        waypoint_count = 0
        boundary_count = 0
        
        for lanelet_id, data in lanelets.items():
            # 1. Waypoints 추가 (노란색)
            if 'waypoints' in data:
                waypoints = data['waypoints'][::waypoint_subsample]  # 서브샘플링
                for wp in waypoints:
                    points.append([wp[0], wp[1], 0.5])  # 살짝 높게 (0.5m)
                    colors.append(self.waypoint_color)
                    waypoint_count += 1
            
            # 2. Left Boundary 추가 (회색)
            if 'leftBound' in data and data['leftBound']:
                for bound_segment in data['leftBound']:
                    boundary_points = bound_segment[::boundary_subsample]  # 서브샘플링
                    for bp in boundary_points:
                        points.append([bp[0], bp[1], 0.0])  # 지면 레벨
                        colors.append(self.boundary_color)
                        boundary_count += 1
            
            # 3. Right Boundary 추가 (회색)
            if 'rightBound' in data and data['rightBound']:
                for bound_segment in data['rightBound']:
                    boundary_points = bound_segment[::boundary_subsample]  # 서브샘플링
                    for bp in boundary_points:
                        points.append([bp[0], bp[1], 0.0])  # 지면 레벨
                        colors.append(self.boundary_color)
                        boundary_count += 1
        
        return self._create_pointcloud2(points, colors)
    
    def _create_pointcloud2(self, points, colors):
        """PointCloud2 메시지 생성"""
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = rospy.Time.now()
        
        # 필드 정의 (XYZ + RGB)
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
        ]
        
        # 데이터 패킹
        cloud_data = []
        for point, color in zip(points, colors):
            # RGB를 uint32로 패킹
            r, g, b = color
            rgb = (r << 16) | (g << 8) | b
            
            # 바이너리 데이터로 패킹
            cloud_data.append(struct.pack('fffI', point[0], point[1], point[2], rgb))
        
        # PointCloud2 메시지 생성
        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1
        pc2.width = len(points)
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 16  # 4 bytes * 4 fields
        pc2.row_step = pc2.point_step * len(points)
        pc2.data = b''.join(cloud_data)
        pc2.is_dense = True
        
        return pc2