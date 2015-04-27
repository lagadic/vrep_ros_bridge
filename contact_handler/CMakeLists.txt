cmake_minimum_required(VERSION 2.8.3)
project(contact_handler)

set(PROJECT_DEPS
    roscpp
    geometry_msgs
    vrep_ros_plugin
    gazebo_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PROJECT_DEPS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PROJECT_DEPS}
  DEPENDS eigen
)

###########
## Build ##
###########

if(NOT VREP_ROOT_DIR)
	set(VREP_ROOT_DIR $ENV{VREP_ROOT_DIR} CACHE PATH "Installation prefix for V-Rep." FORCE)
endif()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${VREP_ROOT_DIR}/programming/include/
)

## Declare a cpp library
add_library(${PROJECT_NAME}
   src/ContactHandler.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)