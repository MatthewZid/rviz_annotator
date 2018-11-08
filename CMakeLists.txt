cmake_minimum_required(VERSION 2.8.3)
project(rviz_annotator)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  pointcloud_msgs
  pcl_ros
  tf
  rviz
)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

#pkg_check_modules(OGRE OGRE)

#include_directories( ${OGRE_INCLUDE_DIRS} )
#link_directories( ${OGRE_LIBRARY_DIRS} )

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

#rosbuild_add_library(${PROJECT_NAME} ${SOURCE_FILES})
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   PointSelection.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs  # Or other packages containing msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp message_runtime
 # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  #${PCL_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

set(SRC_FILES
  src/point_selection.cpp
  src/rviz_rosbag_player.cpp
  src/rviz_control_panel.cpp
  src/rviz_time_translator.cpp
  src/rviz_annotation_panel.cpp
  src/edit_widget.cpp
)

set(HEADER_FILES
  include/${PROJECT_NAME}/point_selection.h
  include/${PROJECT_NAME}/rviz_rosbag_player.h
  include/${PROJECT_NAME}/rviz_control_panel.h
  include/${PROJECT_NAME}/rviz_time_translator.h
  include/${PROJECT_NAME}/rviz_annotation_panel.h
  include/${PROJECT_NAME}/edit_widget.h
)

## Declare a C++ library
#
 add_library(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})
 add_library(pclass src/point_class.cpp include/${PROJECT_NAME}/point_class.h)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} pclass)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(exportBag src/export_bag.cpp)
target_link_libraries(exportBag ${catkin_LIBRARIES} pclass)

#add_executable(stop_display src/stop_display.cpp)
#target_link_libraries(stop_display ${catkin_LIBRARIES}  ${QT_LIBRARIES} ${OGRE_LIBRARIES})

#add_executable(stop_visual src/stop_visual.cpp)
#target_link_libraries(stop_visual ${catkin_LIBRARIES}  ${QT_LIBRARIES} ${OGRE_LIBRARIES})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(exportBag rviz_annotator_generate_messages_cpp)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY config/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)