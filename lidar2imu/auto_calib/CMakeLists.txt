<<<<<<< HEAD
cmake_minimum_required(VERSION 3.5)
project(LidarToImu)
add_definitions(-w)
set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall -lstdc++fs")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
=======
cmake_minimum_required(VERSION 2.8.3)
project(LidarToImu)
set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")
>>>>>>> 6732fdc908e897b074bfced2d0529ae6b5894d8e
## Get Pangolin
find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(PCL REQUIRED)
find_package(Boost REQUIRED system)
find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(ceres)
include_directories(${PROJECT_SOURCE_DIR}/eigen3)
include_directories(${PROJECT_SOURCE_DIR}/ceres/include)

include_directories(${EIGEN_ROOT})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PCL_INCLUDE_DIRS})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE PARSER_PATH src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${PARSER_PATH})
target_link_libraries(${PROJECT_NAME} libjsoncpp.a ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} ceres)

add_executable(run_lidar2imu src/run_lidar2imu.cpp)
target_link_libraries(run_lidar2imu ${PROJECT_NAME})
target_link_libraries(run_lidar2imu ${PCL_LIBRARIES})
<<<<<<< HEAD


add_executable(run_adapt_file src/run_adapt_file.cc)
target_link_libraries(run_adapt_file ${PROJECT_NAME})
target_link_libraries(run_lidar2imu ${PCL_LIBRARIES})

add_executable(run_adapt_pcd_format src/run_adapt_pcd_format.cc)
target_link_libraries(run_adapt_pcd_format ${PROJECT_NAME})
target_link_libraries(run_adapt_pcd_format ${PCL_LIBRARIES})
=======
>>>>>>> 6732fdc908e897b074bfced2d0529ae6b5894d8e
