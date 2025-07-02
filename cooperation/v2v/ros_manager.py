#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from coopmrm.msg import CoopData, TopologyInfo
from std_msgs.msg import Int8
import threading

class ROSManager:
    def __init__(self):
        # 각 차량별 데이터 저장
        self.vehicle_data = {}  # {vehicle_id: {'odom': Odometry, 'path': Path, 'car_state': int}}
        self.publishers = {}    # {vehicle_id: Publisher}
        self.subscribers = {}   # {topic_name: Subscriber}
        
        # 토폴로지 정보 저장
        self.topology_info = {}  # {vehicle_id: TopologyInfo}
        
        # 토픽 모니터링을 위한 스레드
        self.topic_monitor_thread = threading.Thread(target=self.monitor_topics)
        self.topic_monitor_thread.daemon = True
        self.topic_monitor_thread.start()
        
        # 기본 ego 차량 설정
        self.setup_vehicle_topics('ego')
        
        rospy.loginfo("ROS Manager initialized with dynamic topic handling and namespace-based TopologyInfo")
    
    def topology_info_callback(self, msg, vehicle_id):
        """TopologyInfo 메시지 콜백 - 차량별 네임스페이스에서 수신"""
        # 토폴로지 정보 저장
        self.topology_info[vehicle_id] = {
            'generation_level': msg.generation_level,
            'is_assigner': msg.is_assigner,
            'my_children': list(msg.my_children),  # 리스트로 변환
            'max_generation': msg.max_generation,
            'timestamp': rospy.Time.now()
        }
        
        rospy.logdebug(f"Received topology info for {vehicle_id}: "
                      f"Gen:{msg.generation_level}, Assigner:{msg.is_assigner}, "
                      f"Children:{msg.my_children}, MaxGen:{msg.max_generation}")
    
    def get_topology_info(self, vehicle_id):
        """특정 차량의 토폴로지 정보 반환"""
        if vehicle_id in self.topology_info:
            return self.topology_info[vehicle_id]
        else:
            return None
    
    def monitor_topics(self):
        """동적으로 새로운 토픽을 감지하고 구독"""
        rate = rospy.Rate(1)  # 1Hz로 토픽 체크
        
        while not rospy.is_shutdown():
            try:
                # 현재 활성 토픽 목록 가져오기
                topic_list = rospy.get_published_topics()
                
                # target 차량들 찾기 - odom 토픽으로 변경
                for topic_name, topic_type in topic_list:
                    if '/odom' in topic_name and topic_type == 'nav_msgs/Odometry':
                        vehicle_id = self.extract_vehicle_id(topic_name)
                        if vehicle_id and vehicle_id not in self.vehicle_data:
                            self.setup_vehicle_topics(vehicle_id)
                            rospy.loginfo(f"Added new vehicle: {vehicle_id}")
                
                rate.sleep()
            except Exception as e:
                rospy.logwarn(f"Topic monitoring error: {e}")
                rate.sleep()
    
    def extract_vehicle_id(self, topic_name):
        """토픽 이름에서 차량 ID 추출"""
        # /ego/odom -> ego, /target1/odom -> target1
        parts = topic_name.split('/')
        if len(parts) >= 2:
            vehicle_id = parts[1]
            if vehicle_id in ['ego'] or vehicle_id.startswith('target'):
                return vehicle_id
        return None
    
    def setup_vehicle_topics(self, vehicle_id):
        """특정 차량의 토픽 설정"""
        if vehicle_id in self.vehicle_data:
            return
        
        # 데이터 저장소 초기화
        self.vehicle_data[vehicle_id] = {
            'odom': None, 
            'path': None, 
            'car_state': None,
            'pose': None,  # 호환성을 위해 pose 정보도 저장
            'velocity': None  # 속도 정보 별도 저장
        }
        
        # Publisher 생성
        pub_topic = f'/{vehicle_id}/coopdata'
        self.publishers[vehicle_id] = rospy.Publisher(pub_topic, CoopData, queue_size=10)
        
        # Subscribers 생성
        odom_topic = f'/{vehicle_id}/odom'
        path_topic = f'/{vehicle_id}/path'
        car_state_topic = f'/{vehicle_id}/car_state'
        topology_topic = f'/{vehicle_id}/topology_info'  # 네임스페이스별 topology_info
        
        odom_sub = rospy.Subscriber(
            odom_topic, 
            Odometry, 
            lambda msg, vid=vehicle_id: self.odom_callback(msg, vid)
        )
        
        path_sub = rospy.Subscriber(
            path_topic, 
            Path, 
            lambda msg, vid=vehicle_id: self.path_callback(msg, vid)
        )

        car_state_sub = rospy.Subscriber(
            car_state_topic,
            Int8, 
            lambda msg, vid=vehicle_id: self.car_state_callback(msg, vid)
        )
        
        # TopologyInfo 구독 - 차량별 네임스페이스
        topology_sub = rospy.Subscriber(
            topology_topic,
            TopologyInfo,
            lambda msg, vid=vehicle_id: self.topology_info_callback(msg, vid)
        )
        
        self.subscribers[odom_topic] = odom_sub
        self.subscribers[path_topic] = path_sub
        self.subscribers[car_state_topic] = car_state_sub
        self.subscribers[topology_topic] = topology_sub
        
    
    def odom_callback(self, msg, vehicle_id):
        """Odometry 콜백 (차량별)"""
        if vehicle_id in self.vehicle_data:
            self.vehicle_data[vehicle_id]['odom'] = msg
            
            # 호환성을 위해 pose 정보 별도 저장
            self.vehicle_data[vehicle_id]['pose'] = msg.pose.pose
            
            # 속도 정보 별도 저장
            self.vehicle_data[vehicle_id]['velocity'] = {
                'linear_x': msg.twist.twist.linear.x,
                'linear_y': msg.twist.twist.linear.y,
                'angular_z': msg.twist.twist.angular.z,
                'speed': (msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)**0.5
            }
    
    def path_callback(self, msg, vehicle_id):
        """Path 콜백 (차량별)"""
        if vehicle_id in self.vehicle_data:
            self.vehicle_data[vehicle_id]['path'] = msg
    
    def car_state_callback(self, msg, vehicle_id):
        """Car state 콜백 (차량별)"""
        if vehicle_id in self.vehicle_data:
            self.vehicle_data[vehicle_id]['car_state'] = int(msg.data)
    
    def get_latest_odom(self, vehicle_id):
        """특정 차량의 최신 odometry 반환"""
        if vehicle_id in self.vehicle_data:
            return self.vehicle_data[vehicle_id]['odom']
        return None

    def get_latest_pose(self, vehicle_id):
        """특정 차량의 최신 pose 반환 (호환성 유지)"""
        if vehicle_id in self.vehicle_data:
            return self.vehicle_data[vehicle_id]['pose']
        return None
    
    def get_latest_path(self, vehicle_id):
        """특정 차량의 최신 path 반환"""
        if vehicle_id in self.vehicle_data:
            return self.vehicle_data[vehicle_id]['path']
        return None

    def get_latest_car_state(self, vehicle_id):
        """특정 차량의 최신 car state 반환"""
        if vehicle_id in self.vehicle_data:
            return self.vehicle_data[vehicle_id]['car_state']
        return None
    
    def get_latest_velocity(self, vehicle_id):
        """특정 차량의 최신 속도 정보 반환"""
        if vehicle_id in self.vehicle_data:
            return self.vehicle_data[vehicle_id]['velocity']
        return None
    
    def get_vehicle_speed(self, vehicle_id):
        """특정 차량의 속도 크기 반환 (m/s)"""
        velocity = self.get_latest_velocity(vehicle_id)
        if velocity:
            return velocity['speed']
        return 0.0
    
    def get_vehicle_speed_kmh(self, vehicle_id):
        """특정 차량의 속도 크기 반환 (km/h)"""
        speed_ms = self.get_vehicle_speed(vehicle_id)
        return speed_ms * 3.6
    
    def is_vehicle_moving(self, vehicle_id, threshold=0.1):
        """특정 차량이 움직이고 있는지 확인"""
        speed = self.get_vehicle_speed(vehicle_id)
        return speed > threshold
    
    def get_vehicle_position(self, vehicle_id):
        """특정 차량의 위치 [x, y] 반환"""
        pose = self.get_latest_pose(vehicle_id)
        if pose:
            return [pose.position.x, pose.position.y]
        return None
    
    def has_odom_data(self, vehicle_id):
        """특정 차량의 odometry 데이터 존재 여부"""
        return (vehicle_id in self.vehicle_data and 
                self.vehicle_data[vehicle_id]['odom'] is not None)
    
    def has_pose_data(self, vehicle_id):
        """특정 차량의 pose 데이터 존재 여부 (호환성 유지)"""
        return (vehicle_id in self.vehicle_data and 
                self.vehicle_data[vehicle_id]['pose'] is not None)
    
    def has_path_data(self, vehicle_id):
        """특정 차량의 path 데이터 존재 여부"""
        return (vehicle_id in self.vehicle_data and 
                self.vehicle_data[vehicle_id]['path'] is not None)
    
    def has_car_state_data(self, vehicle_id):
        """특정 차량의 car state 데이터 존재 여부"""
        return (vehicle_id in self.vehicle_data and
                self.vehicle_data[vehicle_id]['car_state'] is not None)
    
    def has_velocity_data(self, vehicle_id):
        """특정 차량의 속도 데이터 존재 여부"""
        return (vehicle_id in self.vehicle_data and
                self.vehicle_data[vehicle_id]['velocity'] is not None)
    
    def has_topology_data(self, vehicle_id):
        """특정 차량의 토폴로지 데이터 존재 여부"""
        return vehicle_id in self.topology_info
    
    def publish_coop_data(self, coop_msg, vehicle_id):
        """특정 차량의 CoopData 발행"""
        if vehicle_id in self.publishers:
            self.publishers[vehicle_id].publish(coop_msg)
            rospy.logdebug(f"Published CoopData for vehicle: {vehicle_id}")
    
    def get_active_vehicles(self):
        """현재 활성 차량 목록 반환"""
        return list(self.vehicle_data.keys())
    
    def get_vehicles_with_data(self):
        """데이터가 있는 차량들만 반환"""
        vehicles_with_data = []
        for vehicle_id in self.vehicle_data:
            if self.has_odom_data(vehicle_id):
                vehicles_with_data.append(vehicle_id)
        return vehicles_with_data
    
    def get_vehicles_with_pose_data(self):
        """Pose 데이터가 있는 차량들만 반환 (호환성 유지)"""
        vehicles_with_data = []
        for vehicle_id in self.vehicle_data:
            if self.has_pose_data(vehicle_id):
                vehicles_with_data.append(vehicle_id)
        return vehicles_with_data
    
    def get_vehicle_info(self, vehicle_id):
        """특정 차량의 종합 정보 반환"""
        if vehicle_id not in self.vehicle_data:
            return None
        
        data = self.vehicle_data[vehicle_id]
        position = self.get_vehicle_position(vehicle_id)
        velocity = self.get_latest_velocity(vehicle_id)
        topology = self.get_topology_info(vehicle_id)
        
        return {
            'vehicle_id': vehicle_id,
            'position': position,
            'velocity': velocity,
            'speed_ms': self.get_vehicle_speed(vehicle_id),
            'speed_kmh': self.get_vehicle_speed_kmh(vehicle_id),
            'car_state': data.get('car_state'),
            'is_moving': self.is_vehicle_moving(vehicle_id),
            'topology': topology,
            'has_data': {
                'odom': self.has_odom_data(vehicle_id),
                'path': self.has_path_data(vehicle_id),
                'car_state': self.has_car_state_data(vehicle_id),
                'topology': self.has_topology_data(vehicle_id)
            }
        }
    
    def print_vehicle_status(self):
        """모든 차량 상태 출력 (디버깅용)"""
        print(f"\n=== Vehicle Status ({len(self.vehicle_data)} vehicles) ===")
        
        for vehicle_id in sorted(self.vehicle_data.keys()):
            info = self.get_vehicle_info(vehicle_id)
            if info and info['position']:
                pos = info['position']
                topology = info['topology']
                if topology:
                    print(f"{vehicle_id.upper()}: "
                          f"pos=({pos[0]:.2f}, {pos[1]:.2f}), "
                          f"speed={info['speed_ms']:.2f}m/s ({info['speed_kmh']:.1f}km/h), "
                          f"state={info['car_state']}, "
                          f"gen={topology['generation_level']}, "
                          f"children={topology['my_children']}")
                else:
                    print(f"{vehicle_id.upper()}: "
                          f"pos=({pos[0]:.2f}, {pos[1]:.2f}), "
                          f"speed={info['speed_ms']:.2f}m/s ({info['speed_kmh']:.1f}km/h), "
                          f"state={info['car_state']}, "
                          f"topology=None")
    
    def cleanup(self):
        """정리 작업"""
        # 모든 subscriber 정리
        for subscriber in self.subscribers.values():
            subscriber.unregister()
        
        # 모든 publisher 정리
        for publisher in self.publishers.values():
            publisher.unregister()
        
        self.subscribers.clear()
        self.publishers.clear()
        self.vehicle_data.clear()
        self.topology_info.clear()
        
        rospy.loginfo("ROSManager cleanup completed")

def main():
    rospy.init_node('v2x_coop_data_node', anonymous=True)
    
    try:
        ros_manager = ROSManager()
        rospy.loginfo("V2X CoopData Node started with namespace-based TopologyInfo")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"V2X Node error: {e}")
    finally:
        if 'ros_manager' in locals():
            ros_manager.cleanup()

if __name__ == '__main__':
    main()