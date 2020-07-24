#chassis module data.
########### id of publisher
uint8 id
#chassis_type: 
# 0x00  position mode
# 0x01  swing mode
# 0x02  rand mode
# others define by developer
uint8 type
########
float32 x
float32 v_x
float32 a_x
#########
float32 x_range_min
float32 x_range_max
