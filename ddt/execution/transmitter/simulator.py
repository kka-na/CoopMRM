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

class LegacyVehicle:
    """단순한 kinematic 차량 모델 (폴백용)"""
    
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, vehicle_type='ford_escort'):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.vehicle_type = vehicle_type
        self.L = 2.5  # 기본 wheelbase
        
        # CommonRoad 호환성을 위한 time_step 속성 추가
        self.time_step = 0
        
    def set(self, x, y, yaw):
        """차량 위치 설정"""
        self.x, self.y, self.yaw = x, y, yaw
        self.time_step += 1
    
    def next_state(self, dt, actuator):
        """간단한 kinematic 모델"""
        try:
            # 조향각과 가속도 처리
            steer = actuator.get('steer', 0)
            accel = actuator.get('accel', 0)
            brake = actuator.get('brake', 0)
            
            # 위치 업데이트
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
            self.yaw += self.v * dt * math.tan(steer) / self.L
            self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi
            
            # 속도 업데이트
            if accel > 0 and brake == 0:
                self.v += accel * dt
            elif accel == 0 and brake > 0:
                self.v = max(0, self.v - brake * dt)
            
            self.v = max(0, min(self.v, 30))  # 속도 제한
            self.time_step += 1
            
            return self.x, self.y, self.yaw, self.v
            
        except Exception as e:
            print(f"❌ Legacy 차량 시뮬레이션 오류: {e}")
            return self.x, self.y, self.yaw, self.v

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
            
            # time_step 속성 추가 (호환성)
            self.time_step = 0
            
        except Exception as e:
            print(f"❌ CommonRoad 차량 생성 실패: {e}")
            raise
    
    def set(self, x, y, yaw):
        """차량 위치 설정"""
        self.x, self.y, self.yaw = x, y, yaw
        
        # current_state가 None이 아닌지 확인
        if self.current_state is not None:
            self.current_state = KSState(
                time_step=self.current_state.time_step + 1,
                position=np.array([x, y]),
                velocity=self.v,
                orientation=yaw,
                steering_angle=getattr(self.current_state, 'steering_angle', 0.0)
            )
            self.time_step = self.current_state.time_step
        else:
            print("⚠️ current_state가 None입니다. 재초기화합니다.")
            self.current_state = KSState(
                time_step=0,
                position=np.array([x, y]),
                velocity=self.v,
                orientation=yaw,
                steering_angle=0.0
            )
            self.time_step = 0
    
    def next_state(self, dt, actuator):
        """차량 동역학 시뮬레이션"""
        try:
            # current_state 유효성 검사
            if self.current_state is None:
                print("⚠️ current_state가 None입니다. 재초기화합니다.")
                self.current_state = KSState(
                    time_step=0,
                    position=np.array([self.x, self.y]),
                    velocity=self.v,
                    orientation=self.yaw,
                    steering_angle=0.0
                )
                self.time_step = 0
            
            # 입력 안전 범위 설정
            params = self.vehicle.parameters
            
            # 가속도 계산
            accel_input = actuator.get('accel', 0)
            brake_input = actuator.get('brake', 0)
            acceleration = (accel_input - brake_input) * params.longitudinal.a_max
            
            # 입력 제한
            max_accel = params.longitudinal.a_max
            max_brake = -params.longitudinal.a_max
            acceleration = np.clip(acceleration, max_brake, max_accel)
            
            # 조향각 변화율 (rad/s)
            current_angle = self.current_state.steering_angle
            target_angle = actuator.get('steer', 0)
            angle_diff = target_angle - current_angle
            steer_rate = angle_diff / dt 
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
            
            # next_state 유효성 검사
            if next_state is None:
                print("⚠️ CommonRoad simulate_next_state가 None을 반환했습니다. Legacy 모델로 폴백합니다.")
                return self._fallback_simulation(dt, actuator)
            
            # time_step 속성 확인
            if not hasattr(next_state, 'time_step'):
                print("⚠️ next_state에 time_step 속성이 없습니다. Legacy 모델로 폴백합니다.")
                return self._fallback_simulation(dt, actuator)
            
            # 상태 업데이트
            self.current_state = next_state
            self.x = float(next_state.position[0])
            self.y = float(next_state.position[1])
            self.yaw = float(next_state.orientation)
            self.v = float(max(0, next_state.velocity))
            self.time_step = next_state.time_step
            
            return self.x, self.y, self.yaw, self.v
            
        except Exception as e:
            print(f"⚠️ CommonRoad 시뮬레이션 오류: {e}")
            return self._fallback_simulation(dt, actuator)
    
    def _fallback_simulation(self, dt, actuator):
        """CommonRoad 실패 시 사용할 간단한 kinematic 모델"""
        try:
            steer = actuator.get('steer', 0)
            accel = actuator.get('accel', 0)
            brake = actuator.get('brake', 0)
            
            # 간단한 kinematic 모델
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
            self.yaw += self.v * dt * math.tan(steer) / self.L
            self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi
            
            # 속도 업데이트
            if accel > 0 and brake == 0:
                self.v += accel * dt
            elif accel == 0 and brake > 0:
                self.v = max(0, self.v - brake * dt)
            
            self.v = max(0, min(self.v, 30))
            self.time_step += 1
            
            # ★★ 핵심: current_state 업데이트 ★★
            if self.current_state is not None:
                current_steering = self.current_state.steering_angle
                target_steering = steer
                
                # 조향각 점진적 변화 (15도/초 제한)
                max_change = math.radians(15) * dt  # 0.3도씩 변화
                steering_diff = target_steering - current_steering
                
                if abs(steering_diff) > max_change:
                    new_steering = current_steering + (max_change if steering_diff > 0 else -max_change)
                else:
                    new_steering = target_steering
                
                self.current_state = KSState(
                    time_step=self.time_step,
                    position=np.array([self.x, self.y]),
                    velocity=self.v,
                    orientation=self.yaw,
                    steering_angle=new_steering
                )
                
            return self.x, self.y, self.yaw, self.v
            
        except Exception as e:
            print(f"❌ Fallback 시뮬레이션 오류: {e}")
            return self.x, self.y, self.yaw, self.v


class Simulator:
    def __init__(self, type, map, scenario, x=0, y=0, yaw=0, v=0, vehicle_type='ford_escort'):
        self.type = type
        self.map = map
        self.scenario = scenario
        self.car = {'state':0, 'x': 0, 'y':0,'t': 0,'v': 0}
        self.actuator = {'steer': 0, 'accel': 0, 'brake': 0}
        self.obstacles = []
        
        # 차량 모델 타입 설정 (CommonRoad 사용 가능 여부 확인)
        self.use_commonroad = COMMONROAD_AVAILABLE
        if not self.use_commonroad:
            print("⚠️ CommonRoad 사용 불가, Legacy 모델 사용")

        # 외부에서 전달받은 파라미터로 차량 생성
        self.vehicle = self.create_vehicle(x, y, yaw, v, vehicle_type)
        
        # 차량 생성 검증
        if self.vehicle is None:
            raise ValueError(f"❌ 차량 생성 실패: type={type}, x={x}, y={y}, yaw={yaw}, v={v}")
        
        print(f"✅ 차량 생성 성공: {type} ({vehicle_type}), CommonRoad: {isinstance(self.vehicle, CommonRoadVehicle)}")
        
        self.set_protocol(type)
    
    def create_vehicle(self, x, y, yaw, v=0, vehicle_type='ford_escort'):
        """차량 객체 생성 (CommonRoad 또는 Legacy)"""
        
        # 먼저 CommonRoad 시도
        if self.use_commonroad:
            try:
                vehicle = CommonRoadVehicle(x, y, yaw, v, vehicle_type)
                print(f"✅ CommonRoad 차량 생성 성공: {vehicle_type}")
                return vehicle
            except Exception as e:
                print(f"⚠️ CommonRoad 차량 생성 실패: {e}")
                print("📋 Legacy 모델로 폴백합니다.")
        
        # Legacy 모델로 폴백
        try:
            vehicle = LegacyVehicle(x, y, yaw, v, vehicle_type)
            print(f"✅ Legacy 차량 생성 성공: {vehicle_type}")
            return vehicle
        except Exception as e:
            print(f"❌ Legacy 차량 생성도 실패: {e}")
            return None
    
    def set_protocol(self, type):
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_pose_cb)
        self.vehicle_pub = rospy.Publisher(f'/{type}/pose', PoseStamped, queue_size=1)

    def init_pose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        
        if self.vehicle is not None:
            self.vehicle.set(x, y, yaw)
        else:
            print("⚠️ 차량이 None이어서 위치 설정을 건너뜁니다.")

    def set_actuator(self, msg):
        if len(msg) >= 2:
            self.actuator['steer'] = math.radians(msg[1])
            if msg[0] > 0:
                accel = msg[0]
                brake = 0
            else:
                accel = 0
                brake = min(abs(msg[0]), self.car['v'] / 0.05 if self.car['v'] > 0 else 1.0)
            
            self.actuator['accel'] = accel
            self.actuator['brake'] = brake
        else:
            print(f"⚠️ 잘못된 actuator 메시지: {msg}")
    
    def set_user_input(self, msg):
        scenario = int(msg.get('scenario_number', 1))
        scenario_type = int(msg.get('scenario_type', 1))
        if scenario_type == 2:
            scenario = scenario + 6
        
        # 시나리오가 변경되면 상태만 업데이트 (차량 재생성은 외부에서 처리)
        if self.scenario != scenario:
            self.scenario = scenario
            print(f"시나리오 변경: {self.scenario}")

    def reset_vehicle(self, x, y, yaw, v=0, vehicle_type='ford_escort'):
        """외부에서 차량을 재설정할 때 사용"""
        print(f"🔄 차량 재설정: x={x}, y={y}, yaw={yaw}, v={v}, type={vehicle_type}")
        self.vehicle = self.create_vehicle(x, y, yaw, v, vehicle_type)
        
        if self.vehicle is None:
            print("❌ 차량 재설정 실패")
            return False
        return True

    async def execute(self):
        dt = 0.02
        
        if self.vehicle is not None:
            try:
                # 차량 업데이트
                self.car['x'], self.car['y'], yaw, self.car['v'] = self.vehicle.next_state(dt, self.actuator)
                msg = self.make_posestamped_msg(self.vehicle)
                self.vehicle_pub.publish(msg)
            except Exception as e:
                print(f"❌ 차량 execute 중 오류: {e}")
        else:
            print("⚠️ 차량이 None이어서 execute를 건너뜁니다.")

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