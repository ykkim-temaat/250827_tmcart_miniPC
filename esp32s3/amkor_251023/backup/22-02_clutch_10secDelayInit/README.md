# TMCART

## 펌웨어 기본사양
- esp32-s3
- esp-idf v5.1.2
- micro-ROS Agent
- ros2 humble
- micro_ros_espidf_component (https://github.com/micro-ROS/micro_ros_espidf_component)

## ros2 테스트 명령 - Linear Actuator test

### Z축 정지
temaat@ros2-VM:~/tmcart$ ros2 topic pub --once /linear_actuator_cmd_sub geometry_msgs/msg/Vector3 "{x: 0x10, y: 1.2, z: 3.4}"
publisher: beginning loop
publishing #1: geometry_msgs.msg.Vector3(x=16.0, y=1.2, z=3.4)

### Z축 상승 (후진)
temaat@ros2-VM:~/tmcart$ ros2 topic pub --once /linear_actuator_cmd_sub geometry_msgs/msg/Vector3 "{x: 0x11, y: 1.2, z: 3.4}"
publisher: beginning loop
publishing #1: geometry_msgs.msg.Vector3(x=17.0, y=1.2, z=3.4)


### Z축 하강 (전진)
temaat@ros2-VM:~/tmcart$ ros2 topic pub --once /linear_actuator_cmd_sub geometry_msgs/msg/Vector3 "{x: 0x12, y: 1.2, z: 3.4}"
publisher: beginning loop
publishing #1: geometry_msgs.msg.Vector3(x=18.0, y=1.2, z=3.4)
