import asyncio
import rospy
import sys
import yaml
import signal
from control.control import Control
from transmitter.main import Transmitter
from ros_manager import ROSManager

class Execution():
    def __init__(self, type, car, map):
        rospy.init_node(f'{type}_execution')
        self.type = type
        self.car = car
        self.map = map
        self.scenario = 1  # 기본 시나리오
        
        # YAML에서 차량 설정 로드
        self.vehicle_configs = self.load_vehicle_configs(type, car, map, self.scenario)
        # 각 차량별로 Control과 Transmitter 생성
        self.controllers = []
        self.transmitters = []
        
        for i, config in enumerate(self.vehicle_configs):
            # Control 생성
            ct = Control(car)
            self.controllers.append(ct)
            
            # Transmitter 생성 (모든 차량을 동일하게 처리)
            if i > 0:
                type = f'target{i}'
            tm = Transmitter(type, car, map, 
                           config['x'], config['y'], config['yaw'], 
                           config['v'], config['model_type'])
            self.transmitters.append(tm)
        
        # ROSManager는 전체 차량 수로 초기화
        self.RM = ROSManager(type, len(self.vehicle_configs))
        self.set_values()
    
    def load_vehicle_configs(self, type, car, map, scenario):
        """YAML에서 차량 설정들을 로드"""
        try:
            with open("./config/setting.yaml", "r") as f:
                config = yaml.safe_load(f)
            
            map_config = config.get(map, {})
            scenario_data = map_config.get(scenario, map_config.get("default", {}))
            vehicle_configs = []
            
            # ego 차량 추가
            if type in scenario_data:
                type_data = scenario_data[type]
                ego_pose = type_data.get("ego", [0, 0, 0])  # ego 키 안의 ego 키
                ego_vehicle_type = type_data.get("ego_vehicle_type", "ford_escort")
                
                # 속도 정보 처리
                if len(ego_pose) >= 4:
                    x, y, yaw, v = ego_pose[:4]
                else:
                    x, y, yaw = ego_pose[:3]
                    v = 0
                
                vehicle_configs.append({
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                    'v': v,
                    'model_type': ego_vehicle_type
                })
                
                # target 차량들 추가
                targets_data = scenario_data.get("targets", [])  # scenario_data에서 직접 targets 가져오기
                for i, target_pose in enumerate(targets_data):
                    if len(target_pose) >= 5:
                        x, y, yaw, v, model_type = target_pose[:5]
                    elif len(target_pose) >= 4:
                        x, y, yaw, v = target_pose[:4]
                        model_type = "ford_escort"
                    else:
                        x, y, yaw = target_pose[:3]
                        v = 0
                        model_type = "ford_escort"
                    
                    vehicle_configs.append({
                        'x': x,
                        'y': y,
                        'yaw': yaw,
                        'v': v,
                        'model_type': model_type
                    })
            
            print(f"로드된 차량 수: {len(vehicle_configs)}")
            return vehicle_configs
            
        except Exception as e:
            print(f"⚠️ 차량 설정 로드 실패: {e}, 기본 차량 1대 사용")
            return [{
                'x': 0, 'y': 0, 'yaw': 0, 'v': 0,
                'model_type': 'ford_escort'
            }]

    def set_values(self):
        self.ego = None
        self.targets = None

    def update_values(self):
        self.ego = self.RM.ego
        self.targets = self.RM.targets
        
        # 각 컨트롤러에 해당하는 차량 정보 업데이트
        for i, controller in enumerate(self.controllers):
            if i == 0:
                # 첫 번째는 ego 차량
                controller.update_value(self.RM.target_velocity, self.ego, self.targets)
            else:
                # 나머지는 target 차량들
                target_idx = i - 1
                target = self.targets[target_idx] if target_idx < len(self.targets) else None
                controller.update_value(self.RM.target_velocity, target, [])

    async def control(self):
        while not rospy.is_shutdown():
            self.update_values()
            
            # 각 차량별로 제어 실행
            for i, controller in enumerate(self.controllers):
                actuator, lh = controller.execute()
                #self.RM.pub_lh(lh)
                
                # 해당 transmitter에 actuator 전달
                self.transmitters[i].target.set_actuator(actuator)
                self.transmitters[i].target.set_user_input(self.RM.user_input)
            
            await asyncio.sleep(0.1) #10hz             

    async def run_transmitters(self):
        """모든 transmitter를 병렬로 실행"""
        tasks = []
        for tm in self.transmitters:
            task = asyncio.create_task(tm.transmitter())
            tasks.append(task)
        
        await asyncio.gather(*tasks)

    def execute(self):
        loop = asyncio.get_event_loop()
        control_task = loop.create_task(self.control())
        transmitters_task = loop.create_task(self.run_transmitters())
        
        try:
            loop.run_forever()
        except KeyboardInterrupt:
            print("종료 중...")
            control_task.cancel()
            transmitters_task.cancel()
            loop.stop()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    if len(sys.argv) != 4:
        type = 'ego'
        car = 'simulator'
        map = 'Midan'
    else:
        type = str(sys.argv[1])
        car = str(sys.argv[2])
        map = str(sys.argv[3])
    
    ex = Execution(type, car, map)
    ex.execute()

def signal_handler(sig, frame):
    sys.exit(0)

if __name__ == "__main__":
    main()