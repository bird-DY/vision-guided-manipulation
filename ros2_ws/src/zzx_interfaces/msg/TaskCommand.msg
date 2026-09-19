uint8 TASK_UNKNOWN=0
uint8 TASK_NAVIGATE=1
uint8 TASK_INSPECT=2
uint8 TASK_PICK_PLACE=3
uint8 TASK_PRESS_BUTTON=4
uint8 TASK_TOGGLE_SWITCH=5
uint8 TASK_SORT_OBJECT=6

std_msgs/Header header
string task_id
uint8 task_type

string target_id
int32 target_class_id
string source_location
string destination_location
geometry_msgs/PoseStamped target_pose

bool use_visual_servoing
bool dry_run

# parameter_keys and parameter_values must have the same length.
string[] parameter_keys
string[] parameter_values
