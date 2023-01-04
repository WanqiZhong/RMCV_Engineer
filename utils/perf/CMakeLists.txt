# perf
file(GLOB_RECURSE src
    "src/**"
)

set(link log ${link})

add_library(perf ${src})
target_include_directories(perf PUBLIC include)
target_link_libraries(perf ${link})
