cmake_minimum_required(VERSION 2.8.3)
project(bish_segmentation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
		eigen_conversions
		init_fake_opengl_context
		roscpp
		bish_segmentation_msgs
                pcl_conversions
                eigen_conversions
	)

## System dependencies are found with CMake's conventions

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PCL REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

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
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bish_segmentation_lib
  CATKIN_DEPENDS eigen_conversions init_fake_opengl_context roscpp bish_segmentation_msgs
  DEPENDS Boost Eigen3 GLEW OpenCV OpenGL PCL
)

###########
## Build ##
###########

include_directories(
		include
	  	${Boost_INCLUDE_DIRS}
		${catkin_INCLUDE_DIRS}
		${Eigen_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
		${OPENGL_INCLUDE_DIR}
		${PCL_INCLUDE_DIRS}
	)

add_library(bish_segmentation_lib
		src/ViewGenerator.cpp
		src/surfel_renderer.cpp
		src/shaders/surfel_vertex_shader.cpp
		src/shaders/surfel_fragment_shader.cpp

		src/batchProcessor.cpp
		src/HausdorffDist.cpp
		src/HausdorffImageMatch.cpp
		src/HausdorffImageProcessor.cpp
		src/HausdorffImageSimplify.cpp
		src/SColorMap.cpp
		src/utility.cpp

		src/PointcloudLabeler.cpp
		
		src/evaluation.cpp
		
		#Boykov
#		src/graph.cpp
#		src/maxflow.cpp
	)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(bish_segmentation_exec
		src/main.cpp
	)

add_dependencies(bish_segmentation_exec
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(bish_segmentation_exec
		${Boost_LIBRARIES}
		${catkin_LIBRARIES}
		${Eigen_LIBRARIES}
		${OpenCV_LIBS}
		${PCL_LIBRARIES}
		bish_segmentation_lib
	)

set_target_properties(bish_segmentation_exec
	PROPERTIES OUTPUT_NAME bish_segmentation)

add_executable(bish_segmentation_action_server
		src/action_server.cpp
	)

add_dependencies(bish_segmentation_action_server
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(bish_segmentation_action_server
		${Boost_LIBRARIES}
		${catkin_LIBRARIES}
		${Eigen_LIBRARIES}
		${OpenCV_LIBS}
		${PCL_LIBRARIES}
		bish_segmentation_lib
	)

set_target_properties(bish_segmentation_action_server
	PROPERTIES OUTPUT_NAME action_server)


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

install(TARGETS bish_segmentation_lib
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

## Mark cpp header files for installation
install(DIRECTORY include/bish_segmentation/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		FILES_MATCHING PATTERN "*.h"
	)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bish_segmentation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
