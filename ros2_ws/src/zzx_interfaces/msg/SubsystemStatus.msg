uint8 STATE_UNKNOWN=0
uint8 STATE_READY=1
uint8 STATE_BUSY=2
uint8 STATE_DEGRADED=3
uint8 STATE_ERROR=4
uint8 STATE_ESTOPPED=5

std_msgs/Header header
string name
uint8 state
bool connected
bool enabled
string active_operation
string message
zzx_interfaces/BackendCapabilities capabilities
zzx_interfaces/ExecutionStatus execution

# diagnostic_keys and diagnostic_values must have the same length.
string[] diagnostic_keys
string[] diagnostic_values
