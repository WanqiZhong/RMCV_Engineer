
# spdlog
find_package(spdlog REQUIRED)
set(link args ${link})
set(link spdlog::spdlog ${link})

file(GLOB_RECURSE src
    "src/**"
)

add_library(log ${src})
target_include_directories(log PUBLIC include)
target_link_libraries(log ${link})