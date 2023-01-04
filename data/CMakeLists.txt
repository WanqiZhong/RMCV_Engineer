# OpenCV

find_package(OpenCV REQUIRED)
set(link ${OpenCV_LIBS} ${link})

file(GLOB_RECURSE src
    "src/**"
)

set(link spdlog ${link})

add_library(data ${src})
target_include_directories(data PUBLIC include)
target_link_libraries(data ${link})
