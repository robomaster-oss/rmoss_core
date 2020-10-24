#AutoAimNode info data.
###########
uint8 task_mode
uint8 result_code 
#########target armor info 
int32 armor_id
bool is_big_armor
geometry_msgs/Pose pose
#########extra info
geometry_msgs/Pose[] armors_pose
#########control
float32 pitch
float32 yaw
