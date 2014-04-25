cmake_minimum_required(VERSION 2.8.3)
project(rqt_rosplot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)
catkin_package()
catkin_python_setup()

#Dinstall(FILES plugin.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
