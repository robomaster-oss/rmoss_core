#robot state data.
######## basic info 
uint8 robot_id
uint8 level
uint16 hp
uint16 barrel_heat
uint16 buf_energy
uint8 projectile_num
#######chassis info
float32 current_x
float32 current_y
float32 current_z
####### gimbal info
float32 current_pitch
float32 current_yaw
######## attacked info
uint8 is_attcked
uint8 attacked_armor_id
