array_publisher

source /opt/ros/humble/setup.bash
cd ~/ros2_ws/ws_camera
colcon build --symlink-install
source install/setup.bash
ros2 run camera array_publisher

another terminal

c++で受け取る
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/ws_camera_c
colcon build --symlink-install
source install/setup.bash
ros2 run camera_cpp array_subscriber_cpp

pythonで受け取る
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/ws_camera
colcon build --symlink-install
source install/setup.bash
ros2 run camera array_subscriber
