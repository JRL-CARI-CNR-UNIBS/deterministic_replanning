cmake_minimum_required(VERSION 3.0.2)
project(elastic_strips)

# opzioni di compilazione ottimizzata
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosdyn_core
  roscpp
  tf
  eigen_conversions
  sensor_msgs
  subscription_notifier
  name_sorting
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES elastic_strips
  CATKIN_DEPENDS geometry_msgs roscpp rosdyn_core tf eigen_conversions sensor_msgs subscription_notifier name_sorting
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/elastic_strips.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(node src/node.cpp)
add_dependencies(node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(sim_human src/sim_human.cpp)
add_dependencies(sim_human ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sim_human
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

