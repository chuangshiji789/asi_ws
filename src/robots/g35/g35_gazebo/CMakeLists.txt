# CMakeLists.txt
#
# Copyright (c) 2015 Wunderkammer Laboratory
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5.1)
project(g35_gazebo)

set(ignition-math5_DIR "~/asi_ws/libraries/ignitionrobotics-ign-math/build")

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  tf
  dynamic_reconfigure
  # ignition-math5
)

find_package(ignition-math5 REQUIRED)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++0x")
set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
set(CMAKE_CXX_FLAGS "-std=gnu++11")

generate_dynamic_reconfigure_options(
  config/steering/SteeringParams.cfg
	config/drive/DriveParams.cfg
	config/drive_brake/DriveBrakeParams.cfg
	config/suspension/SuspensionParams.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_ros_g35 
  CATKIN_DEPENDS tf gazebo_ros gazebo_plugins roscpp sensor_msgs
  DEPENDS gazebo
)

include_directories(
  include
	${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ~/asi_ws/libraries/ignitionrobotics-ign-math/build/include/
  ~/asi_ws/libraries/ignitionrobotics-ign-math/include/
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
	${catkin_LIBRARY_DIRS}
  ~/asi_ws/libraries/ignitionrobotics-ign-math/build/include/
  ~/asi_ws/libraries/ignitionrobotics-ign-math/include/
)

add_library(gazebo_ros_g35 plugins/gazebo_ros_g35.cpp)
add_dependencies(gazebo_ros_g35 ${PROJECT_NAME}_gencfg)
target_link_libraries(gazebo_ros_g35 gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})


# target_link_libraries(gazebo_ros_g35
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
#   RayPlugin
# )

# catkin_package()

# catkin_install_python(PROGRAMS scripts/rc_car_controller.py
#                       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

# add_executable(odom_node src/odom_node.cpp)
# target_link_libraries(odom_node ${catkin_LIBRARIES})
