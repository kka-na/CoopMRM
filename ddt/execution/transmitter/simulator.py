#!/usr/bin/python
import tf
import yaml
import numpy as np
import math
import sys
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

# CommonRoad 임포트
try:
    from commonroad_dc.feasibility.vehicle_dynamics import VehicleDynamics, VehicleType
    from commonroad.scenario.state import KSState, InputState
    COMMONROAD_AVAILABLE = True
    
    # 차량 타입 매핑
    VEHICLE_TYPE_MAP = {
        'ford_escort': VehicleType.FORD_ESCORT,
        'bmw_320i': VehicleType.BMW_320i,
        'vw_vanagon': VehicleType.VW_VANAGON,
    }
    
except Exception as e:
    print(f"❌ CommonRoad 임포트 실패: {e}")
    COMMONROAD_AVAILABLE = False
    VEHICLE_TYPE_MAP = {}

def signal_handler(sig, frame):
    sys.exit(0)

class CommonRoadVehicle:
    """CommonRoad 기반 차량 동역학 모델"""
    
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, vehicle_type='ford_escort'):
        if not COMMONROAD_AVAILABLE:
            raise ImportError("CommonRoad 라이브러리가 사용 불가능합니다.")
            
        # 차량 타입 설정
        if vehicle_type not in VEHICLE_TYPE_MAP:
            available_types = list(VEHICLE_TYPE_MAP.keys())
            print(f"경고: '{vehicle_type}' 지원 안함. 사용가능: {available_types}")
            vehicle_type = 'ford_escort'
        
        cr_vehicle_type = VEHICLE_TYPE_MAP[vehicle_type]
        
        try:
            self.vehicle = VehicleDynamics.KS(cr_vehicle_type)
            self.vehicle_type = vehicle_type
            
            # 현재 상태 초기화
            self.current_state = KSState(
                time_step=0,
                position=np.array([x, y]),
                velocity=v,
                orientation=yaw,
                steering_angle=0.0
            )
            
            # 호환성을 위한 속성
            self.x = x
            self.y = y
            self.yaw = yaw
            self.v = v
            self.L = self.vehicle.parameters.a + self.vehicle.parameters.b  # wheelbase
            
            
        except Exception as e:
            print(f"❌ CommonRoad 차량 생성 실패: {e}")
            raise
    
    def set(self, x, y, yaw):
        """차량 위치 설정"""
        self.x, self.y, self.yaw = x, y, yaw
        self.current_state = KSState(
            time_step=self.current_state.time_step + 1,
            position=np.array([x, y]),
            velocity=self.v,
            orientation=yaw,
            steering_angle=self.current_state.steering_angle
        )
    
    def next_state(self, dt, actuator):
        """차량 동역학 시뮬레이션"""
        try:
            # 입력 안전 범위 설정
            params = self.vehicle.parameters
            
            # 가속도 계산
            acceleration = (actuator['accel'] - actuator['brake']) *  params.longitudinal.a_max
            
            # 입력 제한
            max_accel = params.longitudinal.a_max
            max_brake = -params.longitudinal.a_max
            acceleration = np.clip(acceleration, max_brake, max_accel)

            
            # 조향각 변화율 (rad/s)
            steer_rate = actuator['steer'] / dt if dt > 0 else 0
            max_steer_rate = params.steering.v_max
            steer_rate = np.clip(steer_rate, -max_steer_rate, max_steer_rate)
            
            # CommonRoad 입력 형태로 변환
            input_state = InputState(
                time_step=self.current_state.time_step,
                acceleration=acceleration,
                steering_angle_speed=steer_rate
            )
            
            # 차량 동역학 시뮬레이션
            next_state = self.vehicle.simulate_next_state(
                self.current_state, input_state, dt, throw=False
            )
            
            # 상태 업데이트
            self.current_state = next_state
            self.x = float(next_state.position[0])
            self.y = float(next_state.position[1])
            self.yaw = float(next_state.orientation)
            self.v = float(max(0, next_state.velocity))
            
            return self.x, self.y, self.yaw, self.v
            
        except Exception as e:
            print(f"⚠️ CommonRoad 시뮬레이션 오류: {e}")
            # 오류시 간단한 kinematic 모델로 폴백
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
            self.yaw += self.v * dt * math.tan(actuator['steer']) / self.L
            self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi
            
            tar_v = self.v
            if actuator['accel'] > 0 and actuator['brake'] == 0:
                tar_v += actuator['accel'] * dt
            elif actuator['accel'] == 0 and actuator['brake'] >= 0:
                tar_v += -actuator['brake'] * dt
            self.v = max(0, tar_v)
            
            return self.x, self.y, self.yaw, self.v


class Simulator:
    def __init__(self, type, map, scenario):
        self.ego = None
        self.targets = []  # 여러 대의 타겟 차량
        self.type = type
        self.map = map
        self.scenario = scenario
        self.car = {'state':0, 'x': 0, 'y':0,'t': 0,'v': 0}
        self.actuator = {'steer': 0, 'accel': 0, 'brake': 0}
        self.obstacles = []
        
        # 차량 모델 타입 설정 (CommonRoad 사용 가능 여부 확인)
        self.use_commonroad = COMMONROAD_AVAILABLE
        if not self.use_commonroad:
            print("⚠️ CommonRoad 사용 불가")

        self.set_ego()
        self.set_protocol(type)
    
    def create_vehicle(self, x, y, yaw, v=0, vehicle_type='ford_escort'):
        """차량 객체 생성 (CommonRoad 또는 Legacy)"""
        if self.use_commonroad:
            try:
                return CommonRoadVehicle(x, y, yaw, v, vehicle_type)
            except Exception as e:
                print(f"⚠️ CommonRoad 차량 생성 실패 : {e}")
                
    
    def set_protocol(self, type):
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_pose_cb)
        self.ego_pub = rospy.Publisher(f'/{type}/pose', PoseStamped, queue_size=1)
        # 타겟 차량 정보 퍼블리셔 추가
        self.targets_pubs = []
        for i, target in enumerate(self.targets):
            pub = rospy.Publisher(f"/target{i+1}/pose", PoseStamped, queue_size=1)
            self.targets_pubs.append(pub)

    def init_pose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.ego.set(x, y, yaw)

    def set_actuator(self, msg):
        self.actuator['steer'] = math.radians(msg[1])
        if msg[0] > 0:
            accel = msg[0]
            brake = 0
        else:
            accel = 0
            brake = min(abs(msg[0]), self.car['v'] / 0.05)  # 브레이크는 현재 속도까지만 적용
        
        self.actuator['accel'] = accel
        self.actuator['brake'] = brake
    
    def set_user_input(self, msg):
        scenario = int(msg['scenario_number'])
        scenario_type = int(msg['scenario_type'])
        if scenario_type == 2:
            scenario = scenario + 6
        if self.scenario != scenario:
            self.scenario = scenario
            self.set_ego() 

    def set_ego(self):
        """ego 및 타겟 차량들 초기화"""
        with open("./transmitter/config/setting.yaml", "r") as f:
            config = yaml.safe_load(f)
        
        map_config = config.get(self.map, {})
        scenario_data = map_config.get(self.scenario, map_config.get("default", {}))
        type_data = scenario_data.get(self.type, {})
        
        # Ego 차량 설정
        ego_pose = type_data.get("ego", [0, 0, 0])
        ego_vehicle_type = type_data.get("ego_vehicle_type", "ford_escort")
        self.ego = self.create_vehicle(*ego_pose, vehicle_type=ego_vehicle_type)
        
        # 타겟 차량들 설정
        targets_data = scenario_data.get("targets", [])
        # targets가 리스트 형태인 경우
        for i, target_pose in enumerate(targets_data):
            x, y, yaw = target_pose[:3]
            v = target_pose[3] if len(target_pose) > 3 else 0
            vehicle_type = target_pose[4] if len(target_pose) > 4 else "ford_escort"
            target = self.create_vehicle(x, y, yaw, v, vehicle_type)
            self.targets.append(target)
        

    async def execute(self):
        dt = 0.05
        
        # Ego 차량 업데이트
        self.car['x'], self.car['y'], yaw, self.car['v'] = self.ego.next_state(dt, self.actuator)
        msg = self.make_posestamped_msg(self.ego)
        self.ego_pub.publish(msg)

        # 타겟 차량들 업데이트 (간단한 자율 주행 로직 예시)
        for i, target in enumerate(self.targets):
            # 타겟 차량은 일정 속도로 직진하거나 간단한 경로를 따라감
            target_actuator = {
                'steer': 0,  # 직진
                'accel': 0 if target.v < 10 else 0,  # 목표 속도 10m/s
                'brake': 0
            }
            target.next_state(dt, target_actuator)
        
        # 타겟 차량 정보 퍼블리시 (필요시)
        self.publish_targets_info()
    
    def publish_targets_info(self):
        for i, target in enumerate(self.targets):
            msg = self.make_posestamped_msg(target)
            self.targets_pubs[i].publish(msg)

    def make_posestamped_msg(self, vehicle):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"  # RViz fixed_frame과 일치시켜야 보임

        # 위치 설정
        msg.pose.position.x = vehicle.x
        msg.pose.position.y = vehicle.y
        msg.pose.position.z = 0.0

        # yaw → quaternion 변환
        quat = tf.transformations.quaternion_from_euler(0, 0, vehicle.yaw)  # yaw in radians
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        return msg

    def cleanup(self):
        pass