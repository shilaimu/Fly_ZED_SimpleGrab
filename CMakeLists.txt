SET(execName Fly_ZED_SimplrGrab)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
#set (OpenCV_DIR "/home/zl/toolsets/opencv-3.3.0/build")

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

#SET(EXECUTABLE_OUTPUT_PATH ".")
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

SET(VERSION_REQ_OCV "3.1")
SET(CMAKE_CXX_COMPILER "g++")

find_package(ZED 2 REQUIRED)
SET(SPECIAL_OS_LIBS "pthread" "X11")
find_package(OpenCV REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include )

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

SET(SRC_FOLDER src)
#FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")
FILE(GLOB_RECURSE SRC_FILES src/*.cpp)

add_definitions(-std=c++0x -g -O3)

message(${ZED_DIR})
message( ${ZED_LIBRARIES})

#flycapture
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -o3")
find_package(OpenCV ${VERSION_REQ_OCV} REQUIRED)
include_directories("/usr/include/flycapture")
include_directories(
        ${FlyCapture2_INCLUDE_DIR}
	/usr/include/flycapture
)
include_directories(
	"${PROJECT_SOURCE_DIR}/include"
        )
add_executable(${execName}  src/main.cpp  src/flycam.cpp src/zedcam.cpp )
target_link_libraries(${execName}
        ${FlyCapture2_LIBRARIES}        
	${ZED_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${SPECIAL_OS_LIBS}
	/usr/lib/libflycapture.so
)
#flycapture

