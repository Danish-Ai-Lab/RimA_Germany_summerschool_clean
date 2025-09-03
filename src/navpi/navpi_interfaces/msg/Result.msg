uint16 SUCCESS                 =   0
uint16 APPROX_SOLUTION         =   1

uint16 GLOBAL_CANCELED         = 101
uint16 GLOBAL_STOPPED          = 102
uint16 GLOBAL_INVALID_START    = 103
uint16 GLOBAL_INVALID_GOAL     = 104
uint16 GLOBAL_NO_PATH_FOUND    = 105
uint16 GLOBAL_PLANNER_TIMEOUT  = 106
uint16 GLOBAL_TF_ERROR         = 107
uint16 GLOBAL_MAP_ERROR        = 108
uint16 GLOBAL_EXACT_SOLUTION   = 109
uint16 GLOBAL_APPROX_SOLUTION  = 110


uint16 LOCAL_CANCELED          = 201
uint16 LOCAL_STOPPED           = 202
uint16 LOCAL_INVALID_PATH      = 203
uint16 LOCAL_PATH_BLOCKED      = 204
uint16 LOCAL_COLLISION         = 205
uint16 LOCAL_OSCILLATION       = 206
uint16 LOCAL_ROBOT_STUCK       = 207
uint16 LOCAL_EXECUTION_TIMEOUT = 208
uint16 LOCAL_TF_ERROR          = 209
uint16 LOCAL_MAP_ERROR         = 210

# Resulting status of the planner process
uint16 id
# Additional result message
string message
