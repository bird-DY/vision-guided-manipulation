std_msgs/Header header
std_msgs/Header image_header
uint32 image_width
uint32 image_height

string object_id
int32 class_id
string class_name
float32 confidence
string track_id

sensor_msgs/RegionOfInterest roi
geometry_msgs/PoseWithCovariance pose
float32 depth_m
bool depth_valid
bool position_valid
bool orientation_valid
string calibration_id
zzx_interfaces/TargetQuality quality
