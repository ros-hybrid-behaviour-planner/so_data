Header header # standard ROS header

geometry_msgs/Vector3 p  # gradient center

int8 attraction # repulsion / attraction
float32 diffusion # how far information will be spread
float32 goal_radius # size of goal

# evaporation
float32 ev_factor # [0, 1] - 1 means no evaporation, 0 means data lost after ev_time
float32 ev_time # 0 - no evaporation (ev_factor = 1), instant evaporation (ev_factor < 1)

# to define a gradient sector
float32 angle_x
float32 angle_y
geometry_msgs/Vector3 direction

# payload
diagnostic_msgs/KeyValue[] payload
