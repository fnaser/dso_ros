cmake_minimum_required(VERSION 2.4.6)
project(dso_ros)

find_package(
        catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        sensor_msgs
        cv_bridge
        tf
)

set(DSO_PATH $ENV{DSO_PATH})
message("\n\n---- LOOKING FOR DSO at\n\"${DSO_PATH}\"")

find_package(Pangolin 0.2 REQUIRED)
message("\n\n---- FOUND Pangolin Headers at\n\"${Pangolin_INCLUDE_DIRS}\"")
message("---- FOUND Pangolin Libs at\n\"${Pangolin_LIBRARIES}\"")

find_package(OpenCV REQUIRED)
message("\n\n---- FOUND OpenCV Headers at\n\"${OpenCV_INCLUDE_DIRS}\"")
message("---- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

set(
        EIGEN3_INCLUDE_DIR
        "/home/fnaser/Programs/eigen-eigen-b3f3d4950030/"
)
#find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost COMPONENTS system thread)
find_library(DSO_LIBRARY dso ${DSO_PATH}/build/lib)

catkin_package(
        CATKIN_DEPENDS
        nav_msgs
        geometry_msgs
        roscpp
        sensor_msgs
        cv_bridge
        tf
)

###########
## Build ##
###########

set(
        CMAKE_CXX_FLAGS
        #        "${SSE_FLAGS} -O2 -g -std=c++0x -march=native -fno-omit-frame-pointer"
        "${SSE_FLAGS} -O3 -g -std=c++0x -march=native"
)

include_directories(
        include
        ${PROJECT_SOURCE_DIR}/src
        ${DSO_PATH}/src
        ${DSO_PATH}/thirdparty/Sophus
        ${Pangolin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(dso_live src/main.cpp src/ros_output_wrapper.cpp)

target_link_libraries(
        dso_live
        ${DSO_LIBRARY}
        ${Pangolin_LIBRARIES}
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
)

add_executable(ros_pub_imgs src/ros_pub_imgs.cpp)
target_link_libraries(
        ros_pub_imgs
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
)