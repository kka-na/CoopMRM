#!/usr/bin/env python3
"""
Waypoints + Boundary를 하나의 PointCloud2로 색상 구분 시각화
노란색: waypoints, 회색: boundary
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import time
from hdmap.map import MAP


class CombinedPointCloudViz:
    """Waypoints와 Boundary를 하나의 PointCloud로 시각화"""
    
    def __init__(self):
        self.frame_id = 'map'
        self.pointcloud_pub = rospy.Publisher('/map/combined_pointcloud', PointCloud2, queue_size=1, latch=True)
        
        # 색상 정의 (RGB)
        self.waypoint_color = [255, 255, 0]    # 노란색
        self.boundary_color = [128, 128, 128]  # 회색
        
    def create_combined_pointcloud(self, lanelets, waypoint_subsample=3, boundary_subsample=2):
        """
        Waypoints와 Boundary를 하나의 PointCloud2로 생성
        
        Args:
            lanelets: lanelet 데이터
            waypoint_subsample: waypoint 간격 (3 = 3개마다 1개)
            boundary_subsample: boundary 간격 (2 = 2개마다 1개)
        """
        points = []
        colors = []
        
        print(f"🎨 Creating combined PointCloud...")
        print(f"   📍 Waypoints: Yellow")
        print(f"   🔲 Boundaries: Gray")
        
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
        
        print(f"   📊 Waypoints: {waypoint_count:,} points")
        print(f"   📊 Boundaries: {boundary_count:,} points")
        print(f"   📊 Total: {len(points):,} points")
        
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
    
    def publish_and_wait(self, lanelets, duration=10):
        """
        PointCloud 발행하고 지정된 시간 후 종료
        
        Args:
            lanelets: lanelet 데이터
            duration: 대기 시간 (초)
        """
        print(f"🚀 Publishing combined PointCloud...")
        
        # PointCloud 생성 및 발행
        pc_msg = self.create_combined_pointcloud(lanelets)
        self.pointcloud_pub.publish(pc_msg)
        
        print(f"✅ PointCloud published to /map/combined_pointcloud")
        print(f"📺 RViz Settings:")
        print(f"   - Add PointCloud2")
        print(f"   - Topic: /map/combined_pointcloud")
        print(f"   - Fixed Frame: map")
        print(f"   - Size: 3-5 pixels")
        print(f"   - Style: Points")
        
        # 진행 상황 표시하며 대기
        print(f"\n⏳ Waiting {duration} seconds...")
        for i in range(duration):
            remaining = duration - i
            print(f"   🕒 {remaining} seconds remaining...", end='\r')
            time.sleep(1)
        
        print(f"\n🏁 {duration} seconds elapsed. Shutting down...")


class QuickVizNode:
    """빠른 시각화 노드"""
    
    def __init__(self, map_name='Midan', duration=10):
        # ROS 초기화
        rospy.init_node('quick_combined_viz', anonymous=True)
        
        self.map_name = map_name
        self.duration = duration
        self.viz = CombinedPointCloudViz()
        
        print(f"🗺️ Quick Combined PointCloud Visualizer")
        print(f"   Map: {map_name}")
        print(f"   Duration: {duration} seconds")
    
    def run(self):
        """실행"""
        try:
            # 맵 로드
            print(f"📂 Loading {self.map_name} map...")
            midan = MAP(self.map_name)
            print(f"   ✅ Loaded {len(midan.lanelets)} lanelets")
            
            # 시각화 발행 및 대기
            self.viz.publish_and_wait(midan.lanelets, self.duration)
            
        except Exception as e:
            rospy.logerr(f"Failed to visualize map: {e}")
            print(f"❌ Error: {e}")


def main():
    """메인 함수"""
    import sys
    
    # 명령행 인자 처리
    map_name = sys.argv[1] if len(sys.argv) > 1 else 'Midan'
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    
    try:
        node = QuickVizNode(map_name, duration)
        node.run()
        
    except rospy.ROSInterruptException:
        print("\n⏹️ Interrupted by user")
    except KeyboardInterrupt:
        print("\n⏹️ Interrupted by user (Ctrl+C)")
    except Exception as e:
        print(f"❌ Error: {e}")
    
    print("👋 Goodbye!")


if __name__ == '__main__':
    main()