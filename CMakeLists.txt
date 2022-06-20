cmake_minimum_required(VERSION 3.0.2)
project(ximu3_ros)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf2 tf2_ros tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main src/main.cpp)
add_executable(${PROJECT_NAME}_node src/node.cpp)


#get_cmake_property(_variableNames VARIABLES)
#list (SORT _variableNames)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()


find_library(XIMU3 NAMES ximu3 PATHS "${PROJECT_SOURCE_DIR}" NO_DEFAULT_PATH)

if (NOT XIMU3)
    message(WARNING "Please provide the missing library file and then reload the CMake project. https://github.com/xioTechnologies/x-IMU3-Software/blob/main/x-IMU3-API/Rust/target/release/README.md")
endif ()


#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

target_link_libraries(main PRIVATE ${XIMU3} pthread dl m udev)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${XIMU3} pthread dl m udev ${DEPENDENCY_LIBRARIES} ${catkin_LIBRARIES})

