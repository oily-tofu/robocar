cmake_minimum_required(VERSION 3.0.2)
project(tofu)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  gazebo_ros
  urdf
  xacro
  gazebo_ros_control
  gazebo_plugins
  serial
  tf
  geometry_msgs
)

find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tofu
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


# 获取 scripts/ 下所有 .py 文件
file(GLOB PYTHON_SCRIPTS 
    scripts/*.py
)

# 批量安装
catkin_install_python(
    PROGRAMS ${PYTHON_SCRIPTS}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 map 文件夹
install(DIRECTORY
  launch
  config
  map
  gazebo
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
