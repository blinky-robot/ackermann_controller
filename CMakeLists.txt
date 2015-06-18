cmake_minimum_required(VERSION 2.8.3)
project(ackermann_controller)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  dual_controller_interface
  nav_msgs
  tf
)

find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ackermann_controller
  CATKIN_DEPENDS ackermann_msgs dual_controller_interface nav_msgs tf
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(ackermann_controller
  src/ackermann_controller.cpp
)

add_executable(ackermann_controller_test src/ackermann_controller_test.cpp)

target_link_libraries(ackermann_controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(ackermann_controller_test
  ackermann_controller
  ${catkin_LIBRARIES}
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

#############
## Install ##
#############

install(TARGETS ackermann_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
