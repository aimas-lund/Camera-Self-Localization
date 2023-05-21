cd 
gnome-terminal -- MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
echo "PX4 Topics initializated"

# Initialize px4_msgs
cd ~/px4_ros2
colcon build
source install/local_setup.bash

# Initialize camera publisher
cd ~/ros2_ws
colcon build --packages-select A8Camera
source install/local_setup.bash
gnome-terminal -- ros2 run A8Camera camera_talker
echo "Camera_talker initializated"

# Initialze gimbal publisher
gnome-terminal -- ros2 run A8Camera gimbal_talker
echo "Gimbal_talker initializated"
# Initialize streaming
gnome-terminal -- ros2 run A8Camera streamer
echo "Streamer_talker initializated"
