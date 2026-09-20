cmake_minimum_required(VERSION 3.0.2)
project(robot_base_bridge)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs inspection_interfaces)
catkin_package(CATKIN_DEPENDS rospy std_msgs inspection_interfaces)
catkin_install_python(PROGRAMS
  scripts/robot_bridge.py
  scripts/safe_robot_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES scripts/catalog_rules.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

