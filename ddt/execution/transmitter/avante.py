#!/usr/bin/python3
import can
import cantools
import asyncio
import rospy

CAN_INTERFACE = 'socketcan'
CAN_CHANNEL = 'can0'
CAN_BITRATE = 500000
DBC_FILE_PATH = './transmitter/dbc/cn7.dbc'
MAX_ALIVE_COUNT = 256

# 아반떼 CAN ID
EAIT_CONTROL_01_ID = 0x156
EAIT_CONTROL_02_ID = 0x157

TURN_SIGNAL_OFF = 0
TURN_SIGNAL_LEFT = 2
TURN_SIGNAL_RIGHT = 4
TURN_SIGNAL_EMERGENCY = 1

class Avante():
    def __init__(self):
        self._initialize_can_bus()
        self._load_dbc_file()
        self.setup_message_dicts()
        self.setup_encode_handler()

        self.current_velocity = 0.0
        self.alive_cnt = -1
        self.state = 0

    def _initialize_can_bus(self):
        try:
            self.bus = can.ThreadSafeBus(
                interface=CAN_INTERFACE, 
                channel=CAN_CHANNEL, 
                bitrate=CAN_BITRATE
            )
        except Exception as e:
            print(f"CAN 초기화 실패: {e}")
            self.bus = None

    def _load_dbc_file(self):
        self.dbc = cantools.database.load_file(DBC_FILE_PATH)

    def set_actuator(self, msg):
        if self.state == 1:
            self.EAIT_Control_02['ACC_Cmd'] = msg[0]
            self.EAIT_Control_02['EPS_Cmd'] = msg[1]
        else:
            self.EAIT_Control_02['ACC_Cmd'] = -2
    
    def set_user_input(self, msg):
        self.state = int(msg['state'])
        self.EAIT_Control_01['EPS_En'] = self.state
        self.EAIT_Control_01['ACC_En'] = self.state
        
        signal_input = int(msg['signal'])
        if signal_input == 1:
            self.EAIT_Control_01['Turn_Signal'] = TURN_SIGNAL_LEFT
        elif signal_input == 2:
            self.EAIT_Control_01['Turn_Signal'] = TURN_SIGNAL_RIGHT
        elif signal_input == 7:
            self.EAIT_Control_01['Turn_Signal'] = TURN_SIGNAL_EMERGENCY
        else:
            self.EAIT_Control_01['Turn_Signal'] = TURN_SIGNAL_OFF

    def update_alive_cnt(self):
        self.alive_cnt += 1
        if self.alive_cnt >= MAX_ALIVE_COUNT:
            self.alive_cnt = 0
        self.EAIT_Control_01['Aliv_Cnt'] = self.alive_cnt
    
    def update_can_inputs(self):
        self.update_alive_cnt()
        dicts = []
        for values in self.encode_handler.values():
            dicts.append(values)
        return dicts

    async def execute(self):
        if self.bus is None:
            return
            
        dicts = self.update_can_inputs()
        can_messages = self.encode_message(dicts)
        for can_message in can_messages:
            await asyncio.get_event_loop().run_in_executor(None, self.bus.send, can_message)

    def encode_message(self, dicts):
        can_messages = []
        for i, (key, value) in enumerate(self.encode_dbc.items()):
            message = self.dbc.encode_message(value, dicts[i])
            can_message = can.Message(arbitration_id=key, data=message, is_extended_id=False)
            can_messages.append(can_message)
        return can_messages
    
    def setup_encode_handler(self):
        self.encode_handler = {
            EAIT_CONTROL_01_ID: self.EAIT_Control_01,
            EAIT_CONTROL_02_ID: self.EAIT_Control_02,
        }

        self.encode_dbc = {
            EAIT_CONTROL_01_ID: 'EAIT_Control_01',
            EAIT_CONTROL_02_ID: 'EAIT_Control_02'
        }

    def setup_message_dicts(self):
        self.EAIT_Control_01 = {
            'EPS_En': 0x00, 
            'EPS_Override_Ignore': 0x00,  
            'EPS_Speed': 50,  
            'ACC_En': 0x00,  
            'AEB_En': 0x00,  
            'Turn_Signal': 0x00,  
            'AEB_decel_value': 0, 
            'Aliv_Cnt': 0  
        }

        self.EAIT_Control_02 = {
            'EPS_Cmd': 0,  
            'ACC_Cmd': 0 
        }
    
    def cleanup(self):
        if self.bus:
            self.bus.shutdown()