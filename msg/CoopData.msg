Header header

# Vehicle Info
string vehicle_id
geometry_msgs/Pose pose
uint8 generation_level
bool is_fallback

# Path Info  
geometry_msgs/Point[] waypoints
uint8 trust_score

# Control/Status
uint8 phase            # PROPAGATE, REFINE, FUSE
uint8 message_type     # REQUEST, RESPONSE, UPDATE

# 추가 필드 (Network Topology용)
bool is_assigner                    # 현재 assigner 역할 여부
float64 perception_radius           # Coverage diversity 계산용 인지 반경
uint8 max_generation               # 전체 네트워크 최대 세대 수
string[] my_children               # 내가 할당한 자식 차량들