
include_directories(${PROJECT_SOURCE_DIR}/include) 

set(SRCS
   v_repExtRosBridge.cpp
   porting.cpp
   access.cpp
   GenericObjectContainer.cpp
   GenericObjectHandler.cpp
   ConsoleHandler.cpp
   ${VREP_ROOT_DIR}/programming/common/v_repLib.cpp
	)
    

add_library(v_repExtRosBridge SHARED ${SRCS})
target_link_libraries(v_repExtRosBridge ${catkin_LIBRARIES})
SET_TARGET_PROPERTIES(v_repExtRosBridge PROPERTIES COMPILE_FLAGS "-fopenmp")

#install(TARGETS v_repExtRosBridge
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(TARGETS v_repExtRosBridge LIBRARY DESTINATION ${VREP_ROOT_DIR})
#install(TARGETS v_repExtTeleKyb LIBRARY DESTINATION "./" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
