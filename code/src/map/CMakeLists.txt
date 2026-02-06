cmake_minimum_required(VERSION 3.22)
project(test_map_x86)

set(BUILD_PLATFORM "x86")  # 或用命令行传递

find_package(OpenCV REQUIRED)

set(map_path ${CMAKE_CURRENT_LIST_DIR}/../../../)
message(STATUS "map_path = ${map_path}")

file(GLOB MAP_SOURCES
    ${map_path}/code/src/map/src/common_functions.cpp
    ${map_path}/code/src/map/src/lidar_ai_data.cpp
    ${map_path}/code/src/map/src/map_global.cpp
    ${map_path}/code/src/map/src/map_local.cpp
    ${map_path}/code/src/map/src/map_system.cpp
    ${map_path}/code/src/map/src/data_frame.cpp
    ${map_path}/code/src/map/src/data_lidar.cpp
)
add_library(map SHARED ${MAP_SOURCES})
# target_compile_definitions(map PRIVATE USE_DLOG)

target_include_directories(map PRIVATE
    ${map_path}/code/src/map/include
    ${OpenCV_INCLUDE_DIRS}
    ${map_path}/code/include
)

add_executable(${PROJECT_NAME} test_map.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${map_path}/code/include
    ${map_path}/code/src/map/include
    )
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBRARIES} map)
# target_compile_definitions(${PROJECT_NAME} PRIVATE USE_DLOG)
