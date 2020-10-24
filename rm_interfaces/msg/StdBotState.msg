#robot state data.
# basic info 
uint8 id
uint8 level
uint16 hp
uint16 shooter_heat
uint16 buf_energy
uint8 projectile_num

# current chassis and gimbal (position) info
geometry_msgs/Pose chassis
Gimbal2D gimbal

# attacked info
uint8 is_attcked
uint8 attacked_armor_id 
