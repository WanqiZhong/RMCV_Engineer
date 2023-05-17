find_package(spdlog REQUIRED)
file(GLOB_RECURSE src
    "src/**"
)
set(link spdlog::spdlog ${link})

include_directories(include)
add_library(config ${src})
target_link_libraries(config ${link})