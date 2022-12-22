find_package(spdlog REQUIRED)
find_package(toml11 REQUIRED)

file(GLOB_RECURSE src
    "src/**"
)
set(link spdlog::spdlog ${link})
set(link data ${link})

add_library(args ${src})
target_include_directories(args PUBLIC include)
target_link_libraries(args ${link})