Header header
# True if the ball was detected
bool is_detected
# Area of the detected ball in pixels
float32 area
# Radius of the detected ball in pixels
float32 radius
# The 2D location where the ball was detected in the iamge
geometry_msgs/Point32 pos_image
# The 3D location of the ball in the camera frame
geometry_msgs/Point32 pos_camera
# The 3D location of the ball in the robot frame
geometry_msgs/Point32 pos_robot
# 0 to 1 certainty in the detection
float32 certainty


