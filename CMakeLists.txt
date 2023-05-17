cmake_minimum_required(VERSION 3.13)
project(Engineer)

set(CMAKE_CXX_STANDARD 17)
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})

if(NOT ROOT)
    set(ROOT ../)
endif()

set(CMAKE_C_COMPILER_LAUNCHER ccache)
set(CMAKE_CXX_COMPILER_LAUNCHER ccache)

#for perf
add_definitions(-DPERF)
# set for spdlog external fmt
# add_definitions(-DSPDLOG_FMT_EXTERNAL)
# set config path
add_definitions(-DROOT="${ROOT}")

find_package(OpenCV REQUIRED PATHS /usr/include/opencv4)
link_directories(${OpenCV_LIBRARY_DIRS})
find_package(Eigen3 REQUIRED)
add_compile_options(-g) 
# add_link_options(-fsanitize=address)

add_executable(run main.cpp)

add_subdirectory(args)
target_link_libraries(run args)

add_subdirectory(data)
target_link_libraries(run data)

add_subdirectory(utils)
target_link_libraries(run umt)

add_subdirectory(sensor)
target_link_libraries(run sensor)

add_subdirectory(detect)
target_link_libraries(run detect)

add_subdirectory(calculate)
target_link_libraries(run calculate)

add_subdirectory(com)
target_link_libraries(run com)


