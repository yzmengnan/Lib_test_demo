#请在路径内添加以下库
#1 opencv库，可以使用msys2 pacman -S mingw-w64-ucrt-x86_64-opencv 安装后，添加系统环境变量 ucrt的路径
#2 Armadillo库
#3 RL库，RL库的安装方法请看 https://www.roboticslibrary.org/tutorials/build-mingw-w64/
#4 libInterpolate库 https://github.com/CD3/libInterpolate
#5 dvpcamera库 以及twincat运行库请直接连接
cmake_minimum_required(VERSION 3.26)
PROJECT(FINAL)
set(CMAKE_CXX_STANDARD 20)
find_package(EIGEN3 3.4 REQUIRED)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_EXE_LINKER_FLAGS "-static")
include_directories(inc)
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})
find_package(RL COMPONENTS MDL REQUIRED)
find_package(libInterpolate REQUIRED)
aux_source_directory(src src_list)
message(STATUS "BUILD RUNNING LIBRARY")
add_library(runninglib ${src_list})
target_link_libraries(runninglib libInterpolate::Interpolate ${ARMADILLO_LIBRARIES} ${RL_LIBRARIES})
add_executable(trajTest histroy_app/trajTest.cpp)
add_compile_definitions(USE_EIGEN)
add_compile_definitions(COST_TIME=80)
add_compile_definitions(DEFAULT_SYNC_RPM=1000)
add_compile_definitions(SOCKET_PORT=10001)
add_compile_definitions(USE_RL)
if (DEFINED ENV{isSocket})
    message(STATUS "socket needed")
    add_executable(Lib_Demo main.cpp)
    target_link_libraries(Lib_Demo
            ws2_32
            runninglib
            ${PROJECT_SOURCE_DIR}/lib/TcAdsDll.lib
    )
    add_executable(SAFE-CHECK safeCheck.cpp)
    target_link_libraries(SAFE-CHECK
            ws2_32
            runninglib
            ${PROJECT_SOURCE_DIR}/lib/TcAdsDll.lib
    )
endif ()
if (DEFINED ENV{isNoSocket})
    message(STATUS "build no socket program target")
    add_compile_definitions(libtest)
    add_executable(SAFE-CHECK safeCheck.cpp)
    target_link_libraries(SAFE-CHECK
            ws2_32
            runninglib
            ${PROJECT_SOURCE_DIR}/lib/TcAdsDll.lib
    )

    add_executable(noSocketMain main_Nosocket.cpp)
    target_link_libraries(noSocketMain
            ws2_32
            runninglib
            ${PROJECT_SOURCE_DIR}/lib/TcAdsDll.lib
    )
endif ()
if (DEFINED ENV{socketTest})
    message(STATUS "This is SocketTest!")
    add_compile_definitions(SOCKET_TEST)
    add_executable(SocketTest histroy_app/vs_Low.cpp)
    target_link_libraries(SocketTest ws2_32 ${PROJECT_SOURCE_DIR}/lib/TcAdsDll.lib)
endif ()

add_executable(jacobTest jacobTest.cpp)
target_link_libraries(jacobTest ${ARMADILLO_LIBRARIES})
add_executable(TrajTest histroy_app/trajTest.cpp)
target_link_libraries(TrajTest runninglib)
#file(COPY lib/lib_win64/libopenblas.dll DESTINATION ${PROJECT_BINARY_DIR})
#project(VISUAL)
if (DEFINED ENV{OPENCV_NEEDED})
#    set(OpenCV_DIR C:\\opencv\\openCVlocalBuild\\sources\\opencv-4.x\\opencv-4.x\\build)
    find_package(OpenCV REQUIRED)
    if (OpenCV_FOUND)
        message(STATUS "BUILD VISUAL PART")
        add_compile_definitions(visualtest)
        include_directories(${OpenCV_INCLUDE_DIRS})
        message(STATUS "opencv dirs is ${OpenCV_INCLUDE_DIRS}")
        add_executable(VISUAL visualTest.cpp)
        add_executable(DVPTEST histroy_app/Dvp2OpenCV.cpp)
        target_link_libraries(VISUAL
                ws2_32
                runninglib
                ${PROJECT_SOURCE_DIR}/lib/TcAdsDll.lib
                ${PROJECT_SOURCE_DIR}/lib/DVPCamera64.lib
                ${OpenCV_LIBRARIES}
        )
        target_link_libraries(DVPTEST
                ws2_32
                runninglib
                ${PROJECT_SOURCE_DIR}/lib/TcAdsDll.lib
                ${PROJECT_SOURCE_DIR}/lib/DVPCamera64.lib
                ${OpenCV_LIBRARIES}
        )
        add_executable(SAFE-CHECK safeCheck.cpp ${src_list})
        target_link_libraries(SAFE-CHECK ws2_32 runninglib ${PROJECT_SOURCE_DIR}/lib/TcAdsDll.lib)
    else ()
        message(STATUS "NOT FOUND OPENCV")
    endif ()
endif ()
# TODO: check

if (DEFINED ENV{GRAP_TEST})
    message("build grap_app")
    add_executable(GRAP_TEST Grap_Control_Demo.cpp)
    target_link_libraries(GRAP_TEST runninglib ${PROJECT_SOURCE_DIR}/lib/TcAdsDll.lib)
endif()