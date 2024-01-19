cmake_minimum_required(VERSION 3.26)
PROJECT(FINAL)
# TODO: msys2 安装 openblas 简化cmake
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
message("BUILD RUNNING LIBRARY")
add_library(runninglib ${src_list})
target_link_libraries(runninglib libInterpolate::Interpolate ${ARMADILLO_LIBRARIES} ${RL_LIBRARIES})
add_executable(trajTest trajTest.cpp)
add_compile_definitions(USE_EIGEN)
add_compile_definitions(COST_TIME=80)
add_compile_definitions(DEFAULT_SYNC_RPM=1000)
#add_compile_definitions(USE_RL)
if (DEFINED ENV{isSocket})
    message("socket needed")
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
    message("build no socket program target")
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
    message("This is SocketTest!")
    add_compile_definitions(SOCKET_TEST)
    add_executable(SocketTest vs_Low.cpp)
    target_link_libraries(SocketTest ws2_32 ${PROJECT_SOURCE_DIR}/lib/TcAdsDll.lib)
endif ()

add_executable(jacobTest jacobTest.cpp)
target_link_libraries(jacobTest ${ARMADILLO_LIBRARIES})
add_executable(TrajTest trajTest.cpp)
target_link_libraries(TrajTest runninglib)
#file(COPY lib/lib_win64/libopenblas.dll DESTINATION ${PROJECT_BINARY_DIR})
#project(VISUAL)
if (DEFINED ENV{OPENCV_NEEDED})
    set(OpenCV_DIR C:\\opencv\\openCVlocalBuild\\sources\\opencv-4.x\\opencv-4.x\\build)
    find_package(OpenCV)
    if (OpenCV_FOUND)
        message("BUILD VISUAL PART")
        add_compile_definitions(visualtest)
        include_directories(${OpenCV_INCLUDE_DIRS})
        include_directories(inc)
        message("opencv dirs is ${OpenCV_INCLUDE_DIRS}")
        add_executable(VISUAL visualTest.cpp)
        set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs)
        target_link_libraries(VISUAL
                runninglib
                ${PROJECT_SOURCE_DIR}/lib/TcAdsDll.lib
                ${OpenCV_LIBS}
                ${PROJECT_SOURCE_DIR}/lib/DVPCamera64.lib
                ${ARMADILLO_LIBRARIES}
        )
        add_executable(SAFE-CHECK safeCheck.cpp ${src_list})
        target_link_libraries(SAFE-CHECK ws2_32 ${PROJECT_SOURCE_DIR}/lib/TcAdsDll.lib ${OpenBLAS_LIBRARY})
    else ()
        message("NOT FOUND OPENCV")
    endif ()
endif ()
# TODO: check