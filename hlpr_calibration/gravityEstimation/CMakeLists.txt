project(gravestim)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
include_directories(/opt/JACO2SDK/API)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} /usr/lib/x86_64-linux-gnu/libdl.so)






