cmake_minimum_required(VERSION 3.5)

project(nmea_msgs)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
   "msg/Sentence.msg"
   DEPENDENCIES std_msgs
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()