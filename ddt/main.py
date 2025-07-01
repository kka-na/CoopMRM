import subprocess
import sys
import signal
import os
import time

class DDTManager:
    def __init__(self):
        self.processes = []
        
    def signal_handler(self, sig, frame):
        print("\n종료 신호를 받았습니다. 모든 프로세스를 종료합니다...")
        self.terminate_all()
        sys.exit(0)
    
    def terminate_all(self):
        """모든 서브프로세스를 종료"""
        for process in self.processes:
            if process.poll() is None:  # 프로세스가 아직 실행 중인지 확인
                print(f"프로세스 {process.pid} 종료 중...")
                process.terminate()
                try:
                    process.wait(timeout=5)  # 5초 대기
                except subprocess.TimeoutExpired:
                    print(f"프로세스 {process.pid} 강제 종료")
                    process.kill()
    
    def run_decision(self, type='ego', map_name='Midan'):
        """Decision 프로세스 실행"""
        decision_dir = os.path.join(os.getcwd(), 'decision')
        cmd = [sys.executable, 'main.py', type, map_name]
        print(f"Decision 실행: {' '.join(cmd)} (작업 디렉토리: {decision_dir})")
        process = subprocess.Popen(cmd, cwd=decision_dir)
        self.processes.append(process)
        return process
    
    def run_execution(self, type='ego', car='simulator', map='Midan'):
        """Execution 프로세스 실행"""
        execution_dir = os.path.join(os.getcwd(), 'execution')
        cmd = [sys.executable, 'main.py', type, car, map]
        print(f"Execution 실행: {' '.join(cmd)} (작업 디렉토리: {execution_dir})")
        process = subprocess.Popen(cmd, cwd=execution_dir)
        self.processes.append(process)
        return process
    
    def run_all(self, type='ego', car='simulator', map_name='Midan'):
        """Decision과 Execution을 모두 실행"""
        print("=== DDT 시스템 시작 ===")
        
        # Signal handler 등록
        signal.signal(signal.SIGINT, self.signal_handler)
        
        try:
            # Decision 프로세스 시작
            decision_process = self.run_decision(type, map_name)
            time.sleep(1)  # Decision이 먼저 시작되도록 잠시 대기
            
            # Execution 프로세스 시작
            execution_process = self.run_execution(type, car, map_name)
            
            print("모든 프로세스가 시작되었습니다.")
            print("종료하려면 Ctrl+C를 누르세요.")
            
            # 프로세스들이 종료될 때까지 대기
            while True:
                # 프로세스 상태 확인
                decision_alive = decision_process.poll() is None
                execution_alive = execution_process.poll() is None
                
                if not decision_alive:
                    print("⚠️ Decision 프로세스가 종료되었습니다.")
                if not execution_alive:
                    print("⚠️ Execution 프로세스가 종료되었습니다.")
                
                # 둘 다 종료되면 전체 종료
                if not decision_alive and not execution_alive:
                    print("모든 프로세스가 종료되었습니다.")
                    break
                
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\n사용자 중단 요청")
        except Exception as e:
            print(f"오류 발생: {e}")
        finally:
            self.terminate_all()

def main():
    # 명령행 인자 파싱
    if len(sys.argv) == 1:
        # 기본값
        type = 'ego'
        car = 'simulator' 
        map_name = 'Midan'
    elif len(sys.argv) == 2:
        type = sys.argv[1]
        car = 'simulator'
        map_name = 'Midan'
    elif len(sys.argv) == 3:
        type = sys.argv[1]
        car = sys.argv[2]
        map_name = 'Midan'
    elif len(sys.argv) == 4:
        type = sys.argv[1]
        car = sys.argv[2]
        map_name = sys.argv[3]
    else:
        print("사용법: python main.py [type] [car] [map]")
        print("예시: python main.py ego simulator Midan")
        sys.exit(1)
    
    print(f"설정: type={type}, car={car}, map={map_name}")
    
    # DDT 매니저 생성 및 실행
    manager = DDTManager()
    manager.run_all(type, car, map_name)

if __name__ == "__main__":
    main()