import rospy
import sys
import signal

from ros_manager import ROSManager
from localization.hdmap.map  import MAP
from planning.path_planner import PathPlanner

def signal_handler(sig, frame):
    sys.exit(0)

class Decision():
    def __init__(self, type, map_name):
        rospy.init_node(f'{type}_decision')
        
        self.map = MAP(map_name)
        self.pp = PathPlanner(self.map)
        self.RM = ROSManager(type,self.map)
        self.set_values()
    
    def set_values(self):
        pass

    def update_value(self):
        self.pp.update_value(self.RM.current_pose)

    def execute(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.update_value()
            pp_result = self.pp.execute()
            if pp_result is not None:
                self.RM.publish_path(pp_result)
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    if len(sys.argv) != 4 :
        type = 'ego'
        map_name = 'Midan'
    else:
        type = str(sys.argv[1])
        map_name = str(sys.argv[2])
    
    de = Decision(type, map_name)
    de.execute()

if __name__=="__main__":
    main()