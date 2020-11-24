#robot state data.
uint8 robot_id

# from referee_system
uint8 level
uint16 hp
uint16 shooter_heat
uint8 is_attcked
uint8[] attacked_armor_id 

# from robot 
geometry_msgs/Pose chassis_pos
Gimbal gimbal_pos
uint8 projectile_num

