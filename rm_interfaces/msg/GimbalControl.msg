#gimbal module data,include pitch and yaw.
########### id of task node
uint8 task_id
#control type
#  0x00 relative
#  0x01 absolute
#  0x02 local scan
#  0x03 global scan
#  other custom usage
uint8 type
########
float32 yaw
float32 pitch
########
float32 yaw_angular_vel
float32 pitch_angular_vel
########
float32 yaw_scan_min
float32 yaw_scan_max
float32 pitch_scan_min
float32 pitch_scan_max

