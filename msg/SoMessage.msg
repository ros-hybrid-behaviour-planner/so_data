Header header  # standard ROS header
string  parent_frame  # the frame id of the parent frame

geometry_msgs/Vector3 p  # gradient center
geometry_msgs/Quaternion q  # orientation / direction
geometry_msgs/Vector3 direction # initial heading vector

int8 attraction  # repulsion / attraction
float32 diffusion  # how far information will be spread
float32 goal_radius  # size of goal

# evaporation
float32 ev_factor  # [0, 1] - 1 means no evaporation, 0 means data lost after ev_time
float32 ev_time  # 0 - no evaporation (ev_factor = 1), instant evaporation (ev_factor < 1)
time ev_stamp  # evaporation time stamp

# indicates whether data is from a moving object (e.g. robot)
bool moving

# payload encoded as yaml
string payload
