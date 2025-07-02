#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8

class ROSManager:
    def __init__(self, num_targets=3):
        # ROS 초기화
        rospy.init_node('vehicle_control_ui', anonymous=True)
        
        # 차량 네임스페이스 생성
        self.vehicles = ['ego']
        for i in range(1, num_targets + 1):
            self.vehicles.append(f'target{i}')
        
        # 각 차량별 publisher 딕셔너리
        self.publishers = {}
        for vehicle in self.vehicles:
            topic_name = f'/{vehicle}/car_state'
            self.publishers[vehicle] = rospy.Publisher(topic_name, Int8, queue_size=1)
        
        # 명령 코드 정의
        self.COMMANDS = {
            'STOP': 0,
            'GO': 1,
            'FALLBACK': 3
        }
        
        rospy.loginfo(f"ROSManager initialized with vehicles: {self.vehicles}")
        
    def get_vehicles(self):
        """차량 리스트 반환"""
        return self.vehicles.copy()
    
    def get_commands(self):
        """명령 코드 딕셔너리 반환"""
        return self.COMMANDS.copy()
    
    def send_command(self, vehicle, command):
        """개별 차량에 명령 전송"""
        if vehicle not in self.vehicles:
            raise ValueError(f"Unknown vehicle: {vehicle}")
        
        if command not in self.COMMANDS:
            raise ValueError(f"Unknown command: {command}")
        
        command_value = self.COMMANDS[command]
        msg = Int8()
        msg.data = command_value
        
        try:
            self.publishers[vehicle].publish(msg)
            rospy.logdebug(f"Sent {command} ({command_value}) to {vehicle}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to send {command} to {vehicle}: {str(e)}")
            return False
    
    def send_bulk_command(self, command):
        """모든 차량에 일괄 명령 전송"""
        if command not in self.COMMANDS:
            raise ValueError(f"Unknown command: {command}")
        
        command_value = self.COMMANDS[command]
        msg = Int8()
        msg.data = command_value
        
        success_count = 0
        failed_vehicles = []
        
        for vehicle in self.vehicles:
            try:
                self.publishers[vehicle].publish(msg)
                success_count += 1
            except Exception as e:
                rospy.logerr(f"Failed to send {command} to {vehicle}: {str(e)}")
                failed_vehicles.append(vehicle)
        
        rospy.loginfo(f"Bulk {command} sent to {success_count}/{len(self.vehicles)} vehicles")
        
        return {
            'success_count': success_count,
            'total_count': len(self.vehicles),
            'failed_vehicles': failed_vehicles
        }
    
    def is_ros_ok(self):
        """ROS 상태 확인"""
        return not rospy.is_shutdown()
    
    def shutdown(self):
        """ROS 종료"""
        rospy.signal_shutdown("ROSManager shutdown")
        rospy.loginfo("ROSManager shutdown completed")