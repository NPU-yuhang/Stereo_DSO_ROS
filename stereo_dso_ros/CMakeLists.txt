cmake_minimum_required(VERSION 2.6)
project(stereo_dso_ros)

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_module) # added by huicanlin
#set(OpenCV_DIR "/home/huicanlin/Downloads/software/opencv-3.2.0/local/share/OpenCV/")
find_package(OpenCV REQUIRED)
message("\n\n---- FOUND OpenCV Headers at\n\"${OpenCV_INCLUDE_DIRS}\"")
message("---- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")
#set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  cv_bridge
  message_filters
  dso
)

set(STEREO_DSO_PATH ${PROJECT_SOURCE_DIR}/../stereo_dso)
message("\n\n---- LOOKING FOR STEREO_DSO at\n\"${STEREO_DSO_PATH}\"")


find_package(Pangolin 0.2 REQUIRED)
message("\n\n---- FOUND Pangolin Headers at\n\"${Pangolin_INCLUDE_DIRS}\"")
message("---- FOUND Pangolin Libs at\n\"${Pangolin_LIBRARIES}\"")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_module)
#find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread)
#find_library(STEREO_DSO_LIBRARY dso ${STEREO_DSO_PATH}/build/lib)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    sensor_msgs
    cv_bridge
    message_filters
    dso
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O2 -g -std=c++0x -march=native -fno-omit-frame-pointer"
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${STEREO_DSO_PATH}/src
  ${STEREO_DSO_PATH}/thirdparty/Sophus
  ${Pangolin_INCLUDE_DIRS}
  "usr/include/eigen3"
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(stereo_dso_ros src/stereo_dso_ros.cpp)

target_link_libraries(stereo_dso_ros
#    ${STEREO_DSO_LIBRARY}
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
	dso)
