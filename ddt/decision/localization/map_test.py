#!/usr/bin/env python3
"""
원클릭 Midan 맵 시각화
waypoints(노란색) + boundaries(회색)를 하나의 PointCloud로
"""

import rospy
from hdmap.combined_pointcloud_viz import CombinedPointCloudViz
from hdmap.map import MAP
import time


def quick_visualize_midan():
    """Midan 맵 빠른 시각화"""
    print("🗺️ Quick Midan Map Visualization")
    print("=" * 50)
    
    try:
        # ROS 초기화
        print("🚀 Initializing ROS...")
        rospy.init_node('quick_midan_viz', anonymous=True)
        
        # 맵 로드
        print("📂 Loading Midan map...")
        midan = MAP('Midan')
        print(f"   ✅ {len(midan.lanelets)} lanelets loaded")
        
        # 시각화 생성
        print("🎨 Creating visualization...")
        viz = CombinedPointCloudViz()
        
        # 발행
        print("📡 Publishing PointCloud...")
        pc_msg = viz.create_combined_pointcloud(midan.lanelets)
        viz.pointcloud_pub.publish(pc_msg)
        
        print("✅ Published to /map/combined_pointcloud")
        print("\n📺 RViz Quick Setup:")
        print("   1. Open RViz")
        print("   2. Fixed Frame: 'map'")
        print("   3. Add > PointCloud2")
        print("   4. Topic: /map/combined_pointcloud")
        print("   5. Size (Pixels): 4")
        print("   6. Style: Points")
        print("\n🎨 Colors:")
        print("   🟡 Yellow: Waypoints (lane centers)")
        print("   ⚫ Gray: Boundaries (lane edges)")
        
        # 10초 카운트다운
        print(f"\n⏳ Auto-shutdown countdown:")
        for i in range(10, 0, -1):
            print(f"   🕒 {i} seconds...", end='\r', flush=True)
            time.sleep(1)
        
        print(f"\n🏁 Shutting down after 10 seconds.")
        
    except FileNotFoundError:
        print("❌ Midan.json not found!")
        print("   Make sure ./hdmap/maps/Midan.json exists")
    except Exception as e:
        print(f"❌ Error: {e}")


def analyze_before_viz():
    """시각화 전 간단 분석"""
    try:
        midan = MAP('Midan')
        
        # 점 개수 계산
        total_waypoints = sum(len(data['waypoints']) for data in midan.lanelets.values())
        total_left_bounds = sum(sum(len(seg) for seg in data.get('leftBound', [])) 
                              for data in midan.lanelets.values())
        total_right_bounds = sum(sum(len(seg) for seg in data.get('rightBound', [])) 
                               for data in midan.lanelets.values())
        
        print(f"📊 Midan Map Overview:")
        print(f"   Lanelets: {len(midan.lanelets)}")
        print(f"   Waypoints: {total_waypoints:,}")
        print(f"   Left boundaries: {total_left_bounds:,}")
        print(f"   Right boundaries: {total_right_bounds:,}")
        
        # 서브샘플링 후 예상 점 개수
        wp_after = total_waypoints // 3
        lb_after = total_left_bounds // 2
        rb_after = total_right_bounds // 2
        total_after = wp_after + lb_after + rb_after
        
        print(f"\n🎯 After subsampling:")
        print(f"   Waypoints (1/3): {wp_after:,}")
        print(f"   Boundaries (1/2): {(lb_after + rb_after):,}")
        print(f"   Total points: {total_after:,}")
        
        # 메시지 크기 예상
        msg_size = total_after * 16 + 300  # 16 bytes per point + header
        print(f"   Message size: ~{msg_size/1024:.1f} KB")
        
        return True
        
    except Exception as e:
        print(f"❌ Analysis failed: {e}")
        return False


if __name__ == "__main__":
    print("🗺️ Midan Map PointCloud Visualizer")
    print("=" * 60)
    
    # 1. 사전 분석
    if analyze_before_viz():
        print("\n" + "="*60)
        
        # 2. 시각화 실행
        quick_visualize_midan()
    
    print("\n👋 Done!")