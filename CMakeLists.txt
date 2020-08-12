cmake_minimum_required(VERSION 3.5)
project(inter_lidar)

set(CMAKE_PREFIX_PATH /home/lizhi/thirdparty/install)
include_directories(/home/lizhi/thirdparty/include)

set(CMAKE_BUILD_TYPE "RELEASE")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

add_library(interpolation SHARED 
            ${PROJECT_SOURCE_DIR}/src/read_json.cpp
            ${PROJECT_SOURCE_DIR}/src/interpolation.cpp)
#target_sources(inter_lidar PUBLIC ${PROJECT_SOURCE_DIR}/src)

add_executable(lidar_interpolation ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(lidar_interpolation
                        interpolation
                        )

add_executable(data_evaluation ${PROJECT_SOURCE_DIR}/src/data_evaluation.cpp)
target_link_libraries(data_evaluation interpolation)

