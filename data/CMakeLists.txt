# OpenCV

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
set(link ${OpenCV_LIBS} ${link})

file(GLOB_RECURSE src
    "src/**"
)

set(link spdlog ${link})
set(link Eigen3::Eigen ${link})

add_library(data ${src})
target_include_directories(data PUBLIC include)
target_link_libraries(data ${link})
