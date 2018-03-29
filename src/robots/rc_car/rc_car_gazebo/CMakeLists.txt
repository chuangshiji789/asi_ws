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

cmake_minimum_required(VERSION 2.8.3)
project(rc_car_gazebo)
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  tf
)
catkin_package()

catkin_install_python(PROGRAMS scripts/rc_car_controller.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_node src/odom_node.cpp)
target_link_libraries(odom_node ${catkin_LIBRARIES})
