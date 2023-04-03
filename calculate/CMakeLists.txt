file(GLOB_RECURSE src "src/**")
# OpenCV
find_package(OpenCV REQUIRED)
set(link ${OpenCV_LIBS} ${link})

# utils
set(link utils ${link})

# Thread
find_package(Threads REQUIRED)
set(link Threads::Threads ${link})

set(link data ${link})
set(link args ${link})

add_library(calculate ${src})
target_include_directories(calculate PUBLIC include)
target_link_libraries(calculate ${link})
