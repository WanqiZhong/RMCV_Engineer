
find_package(Threads REQUIRED)
set(link Threads::Threads ${link})
set(link args ${link})
set(link data ${link})
set(link log ${link})

set(link utils ${link})

file(GLOB_RECURSE src
    "src/**"
)

add_library(com ${src})
target_include_directories(com PUBLIC include)
target_link_libraries(com ${link})