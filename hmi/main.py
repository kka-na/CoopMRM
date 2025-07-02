#!/usr/bin/env python3

import sys
import tkinter as tk
from tkinter import ttk
import datetime
from ros_manager import ROSManager

class VehicleControlUI:
    def __init__(self, num_targets=3):
        # ROS 매니저 초기화
        try:
            self.ros_manager = ROSManager(num_targets)
            self.vehicles = self.ros_manager.get_vehicles()
            self.commands = self.ros_manager.get_commands()
        except Exception as e:
            print(f"Failed to initialize ROS: {e}")
            sys.exit(1)
        
        # GUI 초기화
        self.setup_gui()
        
    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title(f"ROS Vehicle Control Panel ({len(self.vehicles)} vehicles)")
        self.root.geometry("700x500")
        
        # 창 닫기 이벤트 처리
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 메인 프레임
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 제목
        title_text = f"Vehicle Control Panel - {len(self.vehicles)} Vehicles"
        title_label = ttk.Label(main_frame, text=title_text, 
                               font=('Arial', 16, 'bold'))
        title_label.grid(row=0, column=0, columnspan=4, pady=(0, 20))
        
        # 차량 목록 표시
        vehicles_info = f"Vehicles: {', '.join(self.vehicles)}"
        info_label = ttk.Label(main_frame, text=vehicles_info, 
                              font=('Arial', 10), foreground='gray')
        info_label.grid(row=1, column=0, columnspan=4, pady=(0, 15))
        
        # 일괄 제어 버튼
        bulk_frame = ttk.LabelFrame(main_frame, text="Bulk Control", padding="10")
        bulk_frame.grid(row=2, column=0, columnspan=4, sticky=(tk.W, tk.E), pady=(0, 20))
        
        ttk.Button(bulk_frame, text="ALL GO", 
                  command=lambda: self.send_bulk_command('GO'),
                  style='Go.TButton').grid(row=0, column=0, padx=5)
        
        ttk.Button(bulk_frame, text="ALL STOP", 
                  command=lambda: self.send_bulk_command('STOP'),
                  style='Stop.TButton').grid(row=0, column=1, padx=5)
        
        # 개별 차량 제어
        vehicles_frame = ttk.LabelFrame(main_frame, text="Individual Control", padding="10")
        vehicles_frame.grid(row=3, column=0, columnspan=4, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 스크롤바가 있는 프레임 (차량이 많을 경우를 위해)
        canvas = tk.Canvas(vehicles_frame)
        scrollbar = ttk.Scrollbar(vehicles_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # 헤더
        ttk.Label(scrollable_frame, text="Vehicle", font=('Arial', 10, 'bold')).grid(row=0, column=0, padx=10, pady=5)
        ttk.Label(scrollable_frame, text="GO", font=('Arial', 10, 'bold')).grid(row=0, column=1, padx=10, pady=5)
        ttk.Label(scrollable_frame, text="STOP", font=('Arial', 10, 'bold')).grid(row=0, column=2, padx=10, pady=5)
        ttk.Label(scrollable_frame, text="FALLBACK", font=('Arial', 10, 'bold')).grid(row=0, column=3, padx=10, pady=5)
        ttk.Label(scrollable_frame, text="Status", font=('Arial', 10, 'bold')).grid(row=0, column=4, padx=10, pady=5)
        
        # 상태 표시를 위한 딕셔너리
        self.status_labels = {}
        
        # 각 차량별 제어 버튼
        for i, vehicle in enumerate(self.vehicles, 1):
            # 차량 이름
            ttk.Label(scrollable_frame, text=vehicle, font=('Arial', 10)).grid(row=i, column=0, padx=10, pady=5)
            
            # GO 버튼
            ttk.Button(scrollable_frame, text="GO", 
                      command=lambda v=vehicle: self.send_command(v, 'GO'),
                      style='Go.TButton').grid(row=i, column=1, padx=5, pady=2)
            
            # STOP 버튼
            ttk.Button(scrollable_frame, text="STOP", 
                      command=lambda v=vehicle: self.send_command(v, 'STOP'),
                      style='Stop.TButton').grid(row=i, column=2, padx=5, pady=2)
            
            # FALLBACK 버튼
            ttk.Button(scrollable_frame, text="FALLBACK", 
                      command=lambda v=vehicle: self.send_command(v, 'FALLBACK'),
                      style='Fallback.TButton').grid(row=i, column=3, padx=5, pady=2)
            
            # 상태 라벨
            self.status_labels[vehicle] = ttk.Label(scrollable_frame, text="Ready", 
                                                   foreground="blue")
            self.status_labels[vehicle].grid(row=i, column=4, padx=10, pady=5)
        
        # 스크롤 영역 배치
        canvas.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        # 로그 영역
        log_frame = ttk.LabelFrame(main_frame, text="Command Log", padding="10")
        log_frame.grid(row=4, column=0, columnspan=4, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(20, 0))
        
        self.log_text = tk.Text(log_frame, height=8, width=80)
        log_scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scrollbar.set)
        
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        log_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        # 스타일 설정
        self.setup_styles()
        
        # 그리드 가중치 설정
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(3, weight=1)
        main_frame.rowconfigure(4, weight=1)
        vehicles_frame.columnconfigure(0, weight=1)
        vehicles_frame.rowconfigure(0, weight=1)
        scrollable_frame.columnconfigure(4, weight=1)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
    def setup_styles(self):
        style = ttk.Style()
        
        # GO 버튼 스타일 (녹색)
        style.configure('Go.TButton', foreground='white', background='green')
        
        # STOP 버튼 스타일 (빨간색)
        style.configure('Stop.TButton', foreground='white', background='red')
        
        # FALLBACK 버튼 스타일 (주황색)
        style.configure('Fallback.TButton', foreground='white', background='orange')
    
    def send_command(self, vehicle, command):
        """개별 차량에 명령 전송"""
        try:
            success = self.ros_manager.send_command(vehicle, command)
            if success:
                self.update_status(vehicle, command)
                command_value = self.commands[command]
                self.log_command(f"{vehicle}: {command} ({command_value})")
            else:
                self.log_command(f"Failed to send {command} to {vehicle}")
        except Exception as e:
            self.log_command(f"Error sending {command} to {vehicle}: {str(e)}")
    
    def send_bulk_command(self, command):
        """모든 차량에 일괄 명령 전송"""
        try:
            result = self.ros_manager.send_bulk_command(command)
            command_value = self.commands[command]
            
            # 성공한 차량들의 상태 업데이트
            for vehicle in self.vehicles:
                if vehicle not in result['failed_vehicles']:
                    self.update_status(vehicle, command)
            
            self.log_command(f"BULK {command} sent to {result['success_count']}/{result['total_count']} vehicles ({command_value})")
            
            if result['failed_vehicles']:
                self.log_command(f"Failed vehicles: {', '.join(result['failed_vehicles'])}")
                
        except Exception as e:
            self.log_command(f"Error sending bulk {command}: {str(e)}")
    
    def update_status(self, vehicle, command):
        """차량 상태 업데이트"""
        if vehicle in self.status_labels:
            if command == 'GO':
                self.status_labels[vehicle].config(text="RUNNING", foreground="green")
            elif command == 'STOP':
                self.status_labels[vehicle].config(text="STOPPED", foreground="red")
            elif command == 'FALLBACK':
                self.status_labels[vehicle].config(text="FALLBACK", foreground="orange")
    
    def log_command(self, message):
        """명령 로그 추가"""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}\n"
        
        self.log_text.insert(tk.END, log_message)
        self.log_text.see(tk.END)  # 자동 스크롤
    
    def on_closing(self):
        """창 닫기 시 처리"""
        self.log_command("Shutting down...")
        self.ros_manager.shutdown()
        self.root.destroy()
    
    def run(self):
        """GUI 실행"""
        self.log_command("Vehicle Control UI Started")
        self.log_command(f"Controlling {len(self.vehicles)} vehicles: {', '.join(self.vehicles)}")
        
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.log_command("Interrupted by user")
            self.on_closing()

def main():
    # 명령행 인수 처리
    num_targets = 3  # 기본값
    
    if len(sys.argv) > 1:
        try:
            num_targets = int(sys.argv[1])
            if num_targets < 0:
                print("Error: Number of targets must be non-negative")
                sys.exit(1)
        except ValueError:
            print("Error: Invalid number of targets. Please provide an integer.")
            print("Usage: python3 main.py [number_of_targets]")
            print("Example: python3 main.py 5")
            sys.exit(1)
    
    print(f"Starting Vehicle Control UI with {num_targets} target vehicles...")
    
    try:
        ui = VehicleControlUI(num_targets)
        ui.run()
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()