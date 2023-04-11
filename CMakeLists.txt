cmake_minimum_required(VERSION 3.26)
project(LidarCpp)
set(CMAKE_CXX_STANDARD 20)

find_package(spdlog REQUIRED)
find_package(cppzmq REQUIRED)
find_package(Protobuf REQUIRED)

file(GLOB_RECURSE src "src/**")
file(GLOB_RECURSE proto_files "message/proto/*.proto")

include_directories("include/")
include_directories("thirdparty/Livox-SDK2/include/")

add_subdirectory("thirdparty/Livox-SDK2")
add_executable(LidarCpp ${src} ${proto_files})

protobuf_generate(TARGET LidarCpp LANGUAGE cpp)

include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(LidarCpp spdlog::spdlog cppzmq ${Protobuf_LIBRARIES} livox_lidar_sdk_static)
target_include_directories(LidarCpp PUBLIC ${CMAKE_CURRENT_BINARY_DIR})