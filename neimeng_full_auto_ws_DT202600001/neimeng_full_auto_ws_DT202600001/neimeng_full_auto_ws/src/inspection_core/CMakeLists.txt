cmake_minimum_required(VERSION 3.0.2)
project(inspection_core)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs inspection_interfaces)
catkin_package(CATKIN_DEPENDS rospy std_msgs inspection_interfaces)
catkin_install_python(PROGRAMS
  scripts/inspection_orchestrator.py
  scripts/real_inspection_orchestrator.py
  scripts/production_inspection_orchestrator.py
  scripts/production_inspection_orchestrator_v2.py
  scripts/production_inspection_orchestrator_v4.py
  scripts/production_inspection_orchestrator_final.py
  scripts/full_auto_orchestrator.py
  scripts/map_scoped_full_auto_orchestrator.py
  scripts/real_flow_rules.py
  scripts/yolo_bridge.py
  scripts/gated_yolo_bridge.py
  scripts/start_point_yolo_bridge.py
  scripts/full_auto_yolo_bridge.py
  scripts/result_store.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
