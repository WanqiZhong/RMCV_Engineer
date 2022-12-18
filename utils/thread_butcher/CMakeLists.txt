# thread_butcher
file(GLOB_RECURSE src
    "src/**"
)

set(link log ${link})

add_library(thread_butcher ${src})
target_include_directories(thread_butcher PUBLIC include)
target_link_libraries(thread_butcher ${link})
