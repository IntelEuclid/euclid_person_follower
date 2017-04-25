cmake_minimum_required(VERSION 2.8.3)
project(realsense_person_follower)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
)

generate_dynamic_reconfigure_options(
	cfg/person_follower.cfg
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
  /usr/include 
  /usr/include/eigen3/
  #${LIBS_DIR}/include/pxcinclude	
)
###########
## Build ##
###########




add_library( 	realsense_person_follower_nodelet
  				src/follower_nodelet.cpp
)

## Specify libraries to link a library or executable target against


target_link_libraries( realsense_person_follower_nodelet 
						${catkin_LIBRARIES} 
						${roscpp_LIBRARIES} 						
)

add_dependencies(realsense_person_follower_nodelet ${PROJECT_NAME}_gencfg)


# Install library
install(TARGETS realsense_person_follower_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install xml files
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
