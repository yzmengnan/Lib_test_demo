cmake_minimum_required(VERSION 3.26)
project(SocketTest)
project(SAFE-CHECK)
project(Main)

set(CMAKE_CXX_STANDARD 23)

include_directories(inc)
aux_source_directory(src src_list)

add_executable(Lib_Demo main.cpp ${src_list})
add_executable(SocketTest vs_Low.cpp ${src_list})
add_executable(SAFE-CHECK safeCheck.cpp ${src_list})
target_link_libraries(SocketTest ws2_32 ${PROJECT_SOURCE_DIR}/lib/TcAdsDll.lib)
target_link_libraries(Lib_Demo ws2_32 ${PROJECT_SOURCE_DIR}/lib/TcAdsDll.lib)
target_link_libraries(SAFE-CHECK ws2_32 ${PROJECT_SOURCE_DIR}/lib/TcAdsDll.lib )