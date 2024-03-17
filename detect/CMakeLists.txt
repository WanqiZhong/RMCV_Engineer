file(GLOB_RECURSE src "src/**")
# OpenCV
find_package(OpenCV REQUIRED)
set(link ${OpenCV_LIBS} ${link})

# utils
set(link utils ${link})

# Thread
find_package(Threads REQUIRED)
set(link Threads::Threads ${link})

# find_package(OpenVINO REQUIRED)
# set(link openvino::runtime ${link})
set(include ${InferenceEngine_INCLUDE_DIRS} ${include})
set(include ${ngraph_INCLUDE_DIRS} ${include})
set(link data ${link})
set(link args ${link})
set(link calculate ${link})


add_library(detect ${src})
target_include_directories(detect PUBLIC include)
target_link_libraries(detect ${link})
