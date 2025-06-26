import configparser

class APID:
    def __init__(self, config):
        self.set_configs(config)
        
        # 히스토리 관리 - 4개면 충분 (t-3, t-2, t-1, t)
        self.errors = [0.0, 0.0, 0.0, 0.0]
        self.outputs = [0.0, 0.0, 0.0, 0.0]
        self.controls = [0.0, 0.0, 0.0, 0.0]
        
        # Adaptive PID 파라미터 조정값
        self.dKp, self.dKi, self.dKd = 0.0, 0.0, 0.0
        self.ddKp, self.ddKi, self.ddKd = 0.0, 0.0, 0.0
        
        # 에러 누적용 윈도우
        self.error_history = []
        
        # 수치 안정성을 위한 작은 값
        self.epsilon = 1e-6
    
    def set_configs(self, config):
        pid_config = config['PID']
        self.window_size = int(pid_config['window_size'])
        self.base_Kp = float(pid_config['Kp'])
        self.base_Ki = float(pid_config['Ki']) / self.window_size
        self.base_Kd = float(pid_config['Kd'])
        self.learning_rate = float(pid_config['lr'])
        
        common_config = config['Common']
        self.max_accel = float(common_config['max_accel'])
        self.max_decel = float(common_config['max_decel'])

    def execute(self, state, target_velocity, current_velocity):
        # 차량이 정지 상태면 급제동
        if state < 1:
            return -self.max_decel
                
        # 현재 에러 계산 및 히스토리 업데이트
        current_error = target_velocity - current_velocity
        self._update_history(current_error, current_velocity)
        
        # Adaptive 파라미터 계산
        self._calculate_adaptive_gains()
        self._clip_adaptive_gains()
        
        # 최종 PID 출력 계산
        output = self._calculate_pid_output()
        
        # 출력을 가속/제동 명령으로 변환
        return self._convert_to_actuator_command(output)
    
    def _update_history(self, error, output):
        """히스토리 배열 업데이트 (과거 데이터 시프트)"""
        # 배열 시프트: [0,1,2,3] → [1,2,3,new]
        for i in range(3):
            self.errors[i] = self.errors[i + 1]
            self.outputs[i] = self.outputs[i + 1]
            self.controls[i] = self.controls[i + 1]
        
        # 새로운 값 추가
        self.errors[3] = error
        self.outputs[3] = output
        self.controls[3] = 0.0  # 이번 스텝의 제어값은 아직 계산 전
        
        # 에러 누적 윈도우 관리
        self.error_history.append(error)
        if len(self.error_history) > self.window_size:
            self.error_history.pop(0)

    def _calculate_adaptive_gains(self):
        """Adaptive PID 게인 조정값 계산"""
        # 시스템 응답성 계산 (제어 입력 대비 출력 변화)
        ctrl_change_1 = self.controls[1] - self.controls[0] + self.epsilon
        ctrl_change_2 = self.controls[2] - self.controls[1] + self.epsilon
        
        output_change_1 = self.outputs[1] - self.outputs[0]
        output_change_2 = self.outputs[2] - self.outputs[1]
        
        # 시스템 응답성 기반 적응 계수
        response_factor_1 = output_change_1 / ctrl_change_1
        response_factor_2 = output_change_2 / ctrl_change_2
        
        # ddK 계산 (2스텝 전 데이터 기반)
        adaptation_term_dd = -self.learning_rate * self.errors[2] * response_factor_1
        self.ddKp = adaptation_term_dd * (self.errors[2] - self.errors[3] + self.epsilon)
        self.ddKi = adaptation_term_dd * (self.errors[2] + self.epsilon)
        self.ddKd = adaptation_term_dd * (self.errors[2] - 2*self.errors[1] + self.errors[0] + self.epsilon)
        
        # dK 계산 (1스텝 전 데이터 기반)  
        adaptation_term_d = -self.learning_rate * self.errors[3] * response_factor_2
        self.dKp = adaptation_term_d * (self.errors[3] - self.errors[2] + self.epsilon)
        self.dKi = adaptation_term_d * (self.errors[3] + self.epsilon)
        self.dKd = adaptation_term_d * (self.errors[3] - 2*self.errors[2] + self.errors[1] + self.epsilon)

    def _clip_adaptive_gains(self):
        """적응 게인 조정값을 안전한 범위로 제한"""
        # 기본 게인의 10%를 최대 조정 범위로 설정
        kp_limit = self.base_Kp * 0.1
        ki_limit = self.base_Ki * 0.1  
        kd_limit = self.base_Kd * 0.1
        
        # ddK 제한
        self.ddKp = max(-kp_limit, min(self.ddKp, kp_limit))
        self.ddKi = max(-ki_limit, min(self.ddKi, ki_limit))
        self.ddKd = max(-kd_limit, min(self.ddKd, kd_limit))
        
        # dK 제한
        self.dKp = max(-kp_limit, min(self.dKp, kp_limit))
        self.dKi = max(-ki_limit, min(self.dKi, ki_limit))
        self.dKd = max(-kd_limit, min(self.dKd, kd_limit))

    def _calculate_pid_output(self):
        """최종 PID 출력 계산"""
        # 적응적으로 조정된 최종 게인
        final_Kp = self.base_Kp + self.dKp + self.ddKp
        final_Ki = self.base_Ki + self.dKi + self.ddKi
        final_Kd = self.base_Kd + self.dKd + self.ddKd
        
        # PID 요소 계산
        error = self.errors[3]
        integral = sum(self.error_history)
        derivative = self.errors[3] - self.errors[2]
        
        # PID 출력
        output = (final_Kp * error + 
                 final_Ki * integral + 
                 final_Kd * derivative)
        
        # 제어값 히스토리 업데이트
        self.controls[3] = output
        
        return max(-100, min(output, 100))
    
    def _convert_to_actuator_command(self, output):
        """PID 출력을 실제 액추에이터 명령으로 변환"""
        if output < 0:
            # 제동 명령 (음수)
            return (self.max_decel / 100) * output
        else:
            # 가속 명령 (양수)  
            return (self.max_accel / 100) * output