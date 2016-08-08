cmake_minimum_required(VERSION 2.8.3)
project(vrep_ros_plugin)


if( NOT (CMAKE_BUILD_TYPE STREQUAL "Release") AND NOT (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo"))
	message(WARNING "${PROJECT_NAME} should be compiled in RelWithDebInfo or Release mode")
   #set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
#else()
   #set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  cmake_modules
  rosgraph_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
else()
  set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs#   telekyb_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES v_repExtRosBridge
  CATKIN_DEPENDS roscpp pluginlib
  DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

if(NOT VREP_ROOT_DIR)
	set(VREP_ROOT_DIR $ENV{VREP_ROOT_DIR} CACHE PATH "Installation prefix for V-Rep." FORCE)
endif()
message(STATUS "including ${VREP_ROOT_DIR}/programming/include/")

#set(CMAKE_INSTALL_PREFIX ${VREP_ROOT_DIR} CACHE PATH "Installation prefix for the plugin." FORCE)

file(STRINGS ${VREP_ROOT_DIR}/readme.txt README_TXT LIMIT_COUNT 1)
string(REGEX MATCHALL "V([^\" ]+|\"[^\"]+\")" README_TXT "${README_TXT}")
string(REGEX MATCHALL "[0-9]" README_TXT "${README_TXT}")
list(GET README_TXT 0 VREP_VERSION_MAJOR)
list(GET README_TXT 1 VREP_VERSION_MINOR)
list(GET README_TXT 2 VREP_VERSION_PATCH)

set(PLUGIN_VERSION 1)

message(STATUS "Using vrep version: ${VREP_VERSION_MAJOR}.${VREP_VERSION_MINOR}.${VREP_VERSION_PATCH}")

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/v_repExtRosBridge.h.cmake ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/v_repExtRosBridge.h)


include_directories( include
  ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${VREP_ROOT_DIR}/programming/include/
)

# copy model files in the vrep folder
file(GLOB_RECURSE MODEL_FILES_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} models/*.ttm)
foreach(MODEL_FILE ${MODEL_FILES_SRC})
  get_filename_component(MODEL_FILE_NAME ${MODEL_FILE} NAME_WE)
  list(APPEND MODEL_FILES_DST "${VREP_ROOT_DIR}/${MODEL_FILE}")
  add_custom_command(OUTPUT "${VREP_ROOT_DIR}/${MODEL_FILE}"
	  COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/${MODEL_FILE} ${VREP_ROOT_DIR}/${MODEL_FILE}
	  DEPENDS ${MODEL_FILE} 
    COMMENT "Copying model file ${MODEL_FILE_NAME}")
endforeach(MODEL_FILE)
add_custom_target(copy_models DEPENDS ${MODEL_FILES_DST})

add_subdirectory(src)

## Declare a cpp library
# add_library(vrep_ros_plugin
#   src/${PROJECT_NAME}/vrep_ros_plugin.cpp
# )

## Declare a cpp executable
# add_executable(vrep_ros_plugin_node src/vrep_ros_plugin_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(vrep_ros_plugin_node vrep_ros_plugin_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(vrep_ros_plugin_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
#install(TARGETS v_repExtRosBridge
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vrep_ros_plugin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
