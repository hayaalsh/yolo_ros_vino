cmake_minimum_required(VERSION 2.8.12)
project(yolo_ros_vino)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Boost
find_package(Boost REQUIRED)

# X11
find_package(X11 REQUIRED)
if(X11_FOUND)
	include_directories(${X11_INCLUDE_DIR})
	link_libraries(${X11_LIBRARIES})
endif()

# OpenCV
find_package(OpenCV 3 REQUIRED COMPONENTS opencv_core opencv_imgproc opencv_imgcodecs highgui QUIET)
if(NOT(OpenCV_FOUND))
    message(WARNING "OpenCV is disabled or not found.")
    return()
else()
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

# InferenceEngine
find_package(InferenceEngine QUIET)
if(NOT(InferenceEngine_FOUND))
    message(WARNING "InferenceEngine is disabled or not found.")
    return()
else()
    include_directories(${InferenceEngine_INCLUDE_DIRS})
    include_directories(${InferenceEngine_INCLUDE_DIRS}/../samples/common/)
    include_directories(${InferenceEngine_INCLUDE_DIRS}/../src/extension/)
endif()

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    cv_bridge
    roscpp
    image_transport
    message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

# To declare and build messages, services or actions from within this
# package, follow these steps:
# * Let MSG_DEP_SET be the set of packages whose message types you use in
#   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# * In the file package.xml:
#   * add a build_depend tag for "message_generation"
#   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
#   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
#     but can be declared for certainty nonetheless:
#     * add a exec_depend tag for "message_runtime"
# * In this file (CMakeLists.txt):
#   * add "message_generation" and every package in MSG_DEP_SET to
#     find_package(catkin REQUIRED COMPONENTS ...)
#   * add "message_runtime" and every package in MSG_DEP_SET to
#     catkin_package(CATKIN_DEPENDS ...)
#   * uncomment the add_*_files sections below as needed
#     and list every .msg/.srv/.action file to be processed
#   * uncomment the generate_messages entry below
#   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
add_message_files( FILES
	BoundingBox.msg
	BoundingBoxes.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES
	std_msgs
)


###################################
## catkin specific configuration ##
###################################
# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        cv_bridge
        roscpp
        std_msgs
        image_transport
        message_runtime
    DEPENDS Boost
)


###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
)

# Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

# Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    IE::ie_cpu_extension 
    ${InferenceEngine_LIBRARIES} 
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
