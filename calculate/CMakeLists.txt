file(GLOB calculate_src "src/calculate.cpp")
# OpenCV
find_package(OpenCV REQUIRED)
set(link ${OpenCV_LIBS} ${link})

#Thread
find_package(Threads REQUIRED)
set(link Threads::Threads ${link})

set(link log ${link})
set(link data ${link})
set(link utils ${link})

add_library(calculate ${calculate_src})
target_link_libraries(calculate ${link})