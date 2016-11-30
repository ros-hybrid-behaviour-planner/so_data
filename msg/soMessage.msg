Header header # standard ROS header

Vector p  # gradient center

int32 attraction # repulsion / attraction
float32 diffusion # how far information will be spread
float32 goal_radius # size of goal, should be smaller than diffusion

# to define a gradient sector
float32 angle
Vector direction
