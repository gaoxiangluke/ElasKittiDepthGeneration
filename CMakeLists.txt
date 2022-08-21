cmake_minimum_required(VERSION 3.0.2)
project(depth_generator)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  sensor_msgs
  message_filters
  cv_bridge
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
## Declare a catkin package
catkin_package()
set(ELAS_LIB_SOURCE
  thirdparty/libelas/descriptor.cpp
  thirdparty/libelas/elas.cpp
  thirdparty/libelas/filter.cpp
  thirdparty/libelas/matrix.cpp
  thirdparty/libelas/triangle.cpp
  )
add_library(elas SHARED ${ELAS_LIB_SOURCE})
## Build execs
include_directories(include ${catkin_INCLUDE_DIRS} thirdparty)
add_executable(${PROJECT_NAME} src/depthGenerator.cpp) 
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} elas ${OpenCV_LIBS} )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})