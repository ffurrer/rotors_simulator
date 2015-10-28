cmake_minimum_required(VERSION 2.8.3)
project(rotors_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
# find_package(Eigen3 REQUIRED) # TODO(ff): check if we can definitely remove this....
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

set(CONTROLLER_LIBRARIES lee_position_controller exercise_controller roll_pitch_yawrate_thrust_controller)
catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  LIBRARIES ${CONTROLLER_LIBRARIES}
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

FOREACH(CONTROLLER_LIBRARY ${CONTROLLER_LIBRARIES})
  add_library(${CONTROLLER_LIBRARY}
    src/library/${CONTROLLER_LIBRARY}.cpp
  )
  target_link_libraries(${CONTROLLER_LIBRARY} ${catkin_LIBRARIES})
  add_dependencies(${CONTROLLER_LIBRARY} ${catkin_EXPORTED_TARGETS})

  add_executable(${CONTROLLER_LIBRARY}_node src/nodes/${CONTROLLER_LIBRARY}_node.cpp)
  add_dependencies(${CONTROLLER_LIBRARY}_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${CONTROLLER_LIBRARY}_node
    ${CONTROLLER_LIBRARY} ${catkin_LIBRARIES})
  install(TARGETS ${CONTROLLER_LIBRARY}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

  install(TARGETS ${CONTROLLER_LIBRARY}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
ENDFOREACH()

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
