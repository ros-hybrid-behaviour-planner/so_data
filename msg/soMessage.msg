Header header # standard ROS header

geometry_msgs/Vector3 p  # gradient center

int8 attraction # repulsion / attraction
float32 diffusion # how far information will be spread
float32 goal_radius # size of goal

# evaporation
float32 ev_factor
float32 ev_time

# to define a gradient sector
float32 angle
geometry_msgs/Vector3 direction

# payload
diagnostic_msgs/KeyValue[] payload
