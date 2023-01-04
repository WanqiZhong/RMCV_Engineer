add_subdirectory(umt)

add_library(utils INTERFACE)
target_link_libraries(utils INTERFACE umt)
