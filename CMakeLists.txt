cmake_minimum_required(VERSION 3.15)
project(MySfM02)
find_package(OpenCV 3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system thread REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_STANDARD 14)
set(Boost_USE_STATIC_LIBS ON)


add_executable(MySfM02 main.cpp MyFileUtils.h MyFileUtils.cpp SfMCommon.h SfMCommon.cpp FeatureUtils.h FeatureUtils.cpp MatchingUtils.h MatchingUtils.cpp SfMPipe.h StereoUtils.h StereoUtils.cpp BAUtils.h BAUtils.cpp)

target_link_libraries(MySfM02 ${CERES_LIBRARIES})
target_link_libraries(MySfM02 ${Boost_LIBRARIES})
target_link_libraries(MySfM02 ${OpenCV_LIBS} )

message("${Boost_LIBRARIES} ")
