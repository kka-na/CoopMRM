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