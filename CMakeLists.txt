cmake_minimum_required(VERSION 3.10)
project(Engineer)
set(CMAKE_CXX_STANDARD 17)
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
set(SANITIZE OFF)

set(CMAKE_C_COMPILER_LAUNCHER ccache)
set(CMAKE_CXX_COMPILER_LAUNCHER ccache)

# for code sanitize
if(SANITIZE)
    add_compile_options(-std=c++23 -Ofast -s -flto -march=native
            -fno-unwind-tables -fno-asynchronous-unwind-tables -fno-ident -Wall -Wextra
            -Wpedantic -Wmisleading-indentation -Wunused -Wuninitialized -Wshadow
            -D_GLIBCXX_ASSERTIONS)
endif()

# set config path
add_definitions(-DROOT="${ROOT}")
find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})
find_package(Eigen3 3.3 REQUIRED)

add_executable(run main.cpp)

add_subdirectory(args)
target_link_libraries(run args)

add_subdirectory(data)
target_link_libraries(run data)

add_subdirectory(utils)
target_link_libraries(run log)

add_subdirectory(com)
target_link_libraries(run com)

add_subdirectory(detect)
target_link_libraries(run detect)

add_subdirectory(calculate)
target_link_libraries(run calculate)

add_subdirectory(sensor)
target_link_libraries(run sensor)


