#!/usr/bin/env python3

import rospy
import sys
import signal
import time
from ros_manager import ROSManager
from coop_data_packager import CoopDataPackager

def signal_handler(sig, frame):
    sys.exit(0)


class V2V:
    def __init__(self):
        rospy.init_node('v2v', anonymous=True)
        
        # Initialize components
        self.RM = ROSManager()
        self.packager = CoopDataPackager(self.RM)
        self.rate = rospy.Rate(10)  # 10Hz
        
    def run(self):        
        while not rospy.is_shutdown():
            try:
                # Execute packager
                self.packager.execute()
                self.rate.sleep()
                
            except rospy.ROSInterruptException:
                break
                
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    try:
        
        v2v = V2V()
        v2v.run()
    except rospy.ROSInterruptException:
        pass