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

add_library(sensor ${src} src/uvc.cpp include/uvc.hpp)
target_include_directories(sensor PUBLIC include)
target_link_libraries(sensor ${link})
