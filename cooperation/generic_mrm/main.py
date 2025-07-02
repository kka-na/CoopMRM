#!/usr/bin/env python3

import rospy
import sys
import time
import threading
from ros_manager import ROSManager
from network_topology_construction import NetworkTopologyConstruction

def run_ntc_for_vehicle(vehicle_namespace, num_targets):
    """각 차량별로 별도 스레드에서 NTC 실행 - 동일한 코드, 상황별 동작"""
    ros_manager = ROSManager(
        my_namespace=vehicle_namespace, 
        num_targets=num_targets
    )
    ntc = NetworkTopologyConstruction(ros_manager)
    
    rospy.loginfo(f"Starting NTC for {vehicle_namespace}")
    
    # 초기 대기
    time.sleep(2)
    
    rate = rospy.Rate(2)  # 2 Hz로 더 자주 체크
    while not rospy.is_shutdown():
        try:
            # 모든 차량이 동일한 execute 함수 실행
            # 내부에서 상황에 맞게 알아서 판단
            ntc.execute()
        except Exception as e:
            rospy.logwarn(f"NTC error for {vehicle_namespace}: {e}")
        rate.sleep()

def main():
    # Parse arguments
    num_targets = 6
    if len(sys.argv) > 1:
        try:
            num_targets = int(sys.argv[1])
        except ValueError:
            rospy.logwarn("Invalid number format, using default: 6")
    
    rospy.init_node('network_topology_node', anonymous=True)
    rospy.loginfo(f"Starting NTC for all vehicles with {num_targets} targets - unified code approach")
    
    # 모든 차량 네임스페이스
    vehicle_namespaces = ['/ego'] + [f'/target{i}' for i in range(1, num_targets + 1)]
    
    # 각 차량별로 별도 스레드에서 NTC 실행
    threads = []
    for namespace in vehicle_namespaces:
        thread = threading.Thread(
            target=run_ntc_for_vehicle,
            args=(namespace, num_targets),
            name=f"NTC_{namespace.strip('/')}"
        )
        thread.daemon = True
        thread.start()
        threads.append(thread)
        
        rospy.loginfo(f"Started NTC thread for {namespace} - unified algorithm")
    
    # 메인 스레드에서 대기
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down distributed NTC...")

if __name__ == '__main__':
    main()