cmake_minimum_required(VERSION 2.8.3)
project(wam_joystick)
find_package(catkin REQUIRED COMPONENTS std_msgs std_srvs sensor_msgs geometry_msgs roscpp tf)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)
find_package(Barrett REQUIRED)
link_directories(${BARRETT_LIB_DIRS})
include_directories(${BARRETT_INCLUDE_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

add_executable(wam_joystick src/wam_joystick_demo.cpp)
add_dependencies(wam_joystick ${catkin_EXPORTED_TARGETS} wam_msgs_gencpp)
target_link_libraries(wam_joystick ${BARRETT_LIBRARIES} ${catkin_LIBRARIES})

catkin_package(
    DEPENDS std_msgs std_srvs sensor_msgs geometry_msgs roscpp tf 
)

