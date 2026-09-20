cmake_minimum_required(VERSION 3.0.2)
project(robot_voice_announcer)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/install_voice_files.py
    scripts/check_voice_directory.py
    scripts/check_robot_voice_http.py
    scripts/find_robot_voice_directories.py
    scripts/mqtt_voice_bridge.py
    scripts/publish_voice_package.py
    scripts/robot_voice_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY audio config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
