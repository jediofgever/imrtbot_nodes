cmake_minimum_required(VERSION 2.8.3)
project(imrt_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
)

catkin_package()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

catkin_install_python(PROGRAMS
  nodes/imrt_teleop
  nodes/teleop_triggers
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
