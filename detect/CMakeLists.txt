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

add_library(detect ${src})
target_include_directories(detect PUBLIC include)
target_link_libraries(detect ${link})
