uint8 STATE_UNKNOWN=0
uint8 STATE_ACCEPTED=1
uint8 STATE_VALIDATING=2
uint8 STATE_PLANNING=3
uint8 STATE_EXECUTING=4
uint8 STATE_SETTLING=5
uint8 STATE_SUCCEEDED=6
uint8 STATE_FAILED=7
uint8 STATE_CANCELED=8
uint8 STATE_NEEDS_RECONCILIATION=9

std_msgs/Header header
string operation_id
uint8 state
float32 progress
string stage
zzx_interfaces/ErrorStatus error
