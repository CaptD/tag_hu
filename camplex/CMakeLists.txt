cmake_minimum_required(VERSION 2.8.3)
project(camplex)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
	COMPONENTS		camera_calibration_parsers
					camera_info_manager
					cv_bridge
					image_transport
					message_generation
					nodelet
					roscpp
					sensor_msgs
					std_msgs
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)

# Find OpenCV and set flag for OpenCV3 if needed
find_package(OpenCV 3 REQUIRED)
message( STATUS ${OpenCV_INCLUDE_DIRS} )
message( STATUS ${OpenCV_LIBS} )
add_definitions("-DOPENCV3=1")

# Use userspace libv4l2 for interfacing
find_package(V4L2 REQUIRED)

## Generate service files
add_service_files( 
	FILES 			CaptureFrames.srv
					GetCameraInfo.srv
					PrintCapabilities.srv
					SetStreaming.srv
)

generate_messages(
	DEPENDENCIES	sensor_msgs
					std_msgs
)

catkin_package(
	INCLUDE_DIRS 	include
	
	LIBRARIES 		camplex
					camplex_nodelet
					
	CATKIN_DEPENDS 	camera_calibration_parsers
					camera_info_manager
					cv_bridge 
					image_transport 
					message_runtime 
					nodelet
					roscpp 
					sensor_msgs 
					std_msgs
					
	DEPENDS 		Boost 
					OpenCV 
					system_lib
)

include_directories( include
					 ${Boost_INCLUDE_DIRS}
					 ${OpenCV_INCLUDE_DIRS}
					 ${V4L2_INCLUDE_DIRS}
					 ${catkin_INCLUDE_DIRS}
)

add_library( camplex
	src/CameraCalibration.cpp
	src/CameraDriver.cpp
	src/CameraTypes.cpp
	src/DriverNode.cpp
)
add_dependencies( camplex 
	${camplex_EXPORTED_TARGETS} )
target_link_libraries( camplex
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
	${V4L2_LIBRARIES}
	${catkin_LIBRARIES}
)

add_library( camplex_nodelet 
	nodes/camera_nodelet.cpp )
add_dependencies( camplex_nodelet 
	${camplex_EXPORTED_TARGETS}
)
target_link_libraries( camplex_nodelet 
	camplex
	${catkin_LIBRARIES} 
)

add_executable( camera_node
	nodes/camera_node.cpp )
add_dependencies( camera_node 
	camplex_gencfg )
target_link_libraries( camera_node
	${catkin_LIBRARIES}
	camplex
	${OpenCV_LIBS} )

add_executable( viewer_node 
	nodes/viewer_node.cpp )
add_dependencies( viewer_node 
	camplex_gencfg )
target_link_libraries( viewer_node
	${catkin_LIBRARIES}
	${OpenCV_LIBS} )
	
add_executable( recorder_node 
	nodes/recorder_node.cpp )
add_dependencies( recorder_node 
	camplex_gencfg )
target_link_libraries( recorder_node
	${catkin_LIBRARIES}
	${OpenCV_LIBS} )
	
add_executable( video_recorder 
	nodes/video_recorder.cpp )
add_dependencies( video_recorder 
	camplex_gencfg )
target_link_libraries( video_recorder
	${catkin_LIBRARIES}
	${OpenCV_LIBS} )

add_executable( undistortion_node
	nodes/undistortion_node.cpp )
add_dependencies( undistortion_node
	camplex_gencfg )
target_link_libraries( undistortion_node
    camplex
	${catkin_LIBRARIES}
	${OpenCV_LIBS} )
	
## Mark executables and/or libraries for installation
install(TARGETS camplex camera_node viewer_node recorder_node video_recorder undistortion_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/camplex/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
