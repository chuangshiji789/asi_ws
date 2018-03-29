cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(object_tracking)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lboost_system") # setting the compiler flag to c++11 breaks the removenanfromPointCloud function in PCL
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
add_definitions(${CMAKE_CXX_FLAGS})


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  cmake_modules
	hungarian_algorithm
	visualization_msgs
	tf
  # pcl_ros
  # pcl_msgs
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem thread COMPONENTS)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} munkres
 CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions
 DEPENDS system_lib pcl eigen Boost hungarian_algorithm
)

## Sources
set(kalman_filter_SRCS
    src/kalman.cpp
    include/kalman.hpp
)

set(tracker_SRCS
		src/tracker.cpp
		include/tracker.h)

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${BOOST_INCLUDE_DIRS})
link_directories(${BOOST_LIBRARY_DIR})

## Add serial library
add_library(${PROJECT_NAME} ${kalman_filter_SRCS})
add_library(tracker ${tracker_SRCS})

target_link_libraries(${PROJECT_NAME} ${EIGEN_LIBRARIES})
target_link_libraries(tracker ${EIGEN_LIBRARIES})

# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )
# set(PCL_DIR "/home/vas0006/devel_tools/pcl-pcl-1.8.0/install2/share/pcl-1.8")

# set(OpenCV_DIR "/home/vas0006/devel_tools/opencv_install/share/OpenCV")
# find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)

# include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (kalman_test src/kalman-test.cpp)
add_executable (tracker_test src/tracker-test.cpp)
add_executable (tracker_v0 src/tracker_v0.cpp)
add_executable (tracker_v1 src/tracker_v1.cpp)
add_executable (tracker_v2 src/tracker_v2.cpp)
add_executable (tracker_v3 src/tracker_v3.cpp)
add_executable (tracker_v4 src/tracker_v4.cpp)

# add_executable (segmentation src/segmentation.cpp)
# add_executable (surface_recognition src/surface_recognition.cpp)

# target_link_libraries (passthrough ${PCL_LIBRARIES})
target_link_libraries (kalman_test ${PROJECT_NAME} ${EIGEN_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries (tracker_test ${EIGEN_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} tracker)
target_link_libraries (tracker_v0 ${EIGEN_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} tracker)
target_link_libraries (tracker_v1 ${EIGEN_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} tracker)
target_link_libraries (tracker_v2 ${EIGEN_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} tracker)
target_link_libraries (tracker_v3 ${EIGEN_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} tracker)
target_link_libraries (tracker_v4 ${EIGEN_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} tracker)

# target_link_libraries (segmentation ${PROJECT_NAME} ${EIGEN_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
# target_link_libraries (surface_recognition ${PROJECT_NAME} ${Boost_LIBRARIES} ${EIGEN_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

## Include headers
include_directories(include)
