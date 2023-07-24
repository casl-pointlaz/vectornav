# ImuWithCount.msg
# Custom ROS message to store the IMU values with count

uint32 count                     # Use it to store a count
sensor_msgs/Imu imu             # Contains the imu structure, with a header containing the timestamp
