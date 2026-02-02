cmake_minimum_required(VERSION 3.10)

project(rknn_yolov8_demo)

# ============================================================
# ASAN
# ============================================================
if (ENABLE_ASAN)
    message(STATUS "BUILD WITH ADDRESS SANITIZER")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
endif ()

# ============================================================
# OpenCV (aarch64, RKNN Model Zoo)
# ============================================================
if(ANDROID)
    message(STATUS "Using OpenCV Android arm64-v8a")

    set(OpenCV_DIR
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/opencv/opencv-android-sdk-build/sdk/native/jni/abi-arm64-v8a
    )
else()
    message(STATUS "Using OpenCV Linux")

    set(OpenCV_DIR
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/opencv/opencv-linux-aarch64/share/OpenCV
    )
endif()

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")


# ============================================================
# YOLOV8 source selection by SOC
# ============================================================
set(rknpu_yolov8_file src/rknpu2/yolov8_detect.cc)

if (TARGET_SOC STREQUAL "rv1106" OR TARGET_SOC STREQUAL "rv1103")
    add_definitions(-DRV1106_1103)
    set(rknpu_yolov8_file src/rknpu2/yolov8_rv1106_1103.cc)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/allocator/dma)
endif()

if (TARGET_SOC STREQUAL "rk1808" OR TARGET_SOC STREQUAL "rv1109" OR TARGET_SOC STREQUAL "rv1126")
    add_definitions(-DRKNPU1)
    set(rknpu_yolov8_file src/rknpu1/yolov8.cc)
endif()

# ============================================================
# Third-party & utils
# ============================================================
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/ 3rdparty.out)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../utils/ utils.out)

set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")

# ============================================================
# Executable
# ============================================================
add_executable(${PROJECT_NAME}
    src/main.cc
    src/postprocess.cc
    src/transform_coordinates.cc
    src/hand_detect_interface.cc
    src/event_control.cc
    src/debug.cc
    ${rknpu_yolov8_file}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${LIBRKNNRT_INCLUDES}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    imageutils
    fileutils
    imagedrawing
    ${OpenCV_LIBS}
    ${LIBRKNNRT}
    dl
)

# ============================================================
# Threads / platform
# ============================================================
if (CMAKE_SYSTEM_NAME STREQUAL "Android")
    target_link_libraries(${PROJECT_NAME} log)
endif()

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    target_link_libraries(${PROJECT_NAME} Threads::Threads)
endif()

# ============================================================
# Shared library (inference interface)
# ============================================================
add_library(hand_detect_interface SHARED
    src/hand_detect_interface.cc
    src/postprocess.cc
    src/transform_coordinates.cc
    src/event_control.cc
    src/debug.cc
    ${rknpu_yolov8_file}
)

target_include_directories(hand_detect_interface PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${LIBRKNNRT_INCLUDES}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(hand_detect_interface
    imageutils
    fileutils
    imagedrawing
    ${OpenCV_LIBS}
    ${LIBRKNNRT}
    dl
)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(Threads REQUIRED)
    target_link_libraries(hand_detect_interface Threads::Threads)
endif()

# ============================================================
# Install
# ============================================================
install(TARGETS ${PROJECT_NAME} DESTINATION .)
install(TARGETS hand_detect_interface DESTINATION lib)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../model/hand.jpg DESTINATION model)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../model/labels_list.txt DESTINATION model)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../config/cfg.txt DESTINATION config)

file(GLOB RKNN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../model/*.rknn")
install(FILES ${RKNN_FILES} DESTINATION model)


# # 设置版本号和时间戳
# set(LIBRARY_VERSION "HandDetect:1.0.0")
# execute_process(COMMAND date "+%Y-%m-%d_%H-%M-%S" OUTPUT_VARIABLE CMAKE_DATE)
# string(STRIP ${CMAKE_DATE} CMAKE_DATE)
# set(BUILD_TIMESTAMP "${CMAKE_DATE}")

# # 生成 version.h 文件，替换 version.h.in 中的占位符
# configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/version.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/version.h)

# 查看此动态库版本信息的命令：nm -gC lib/libhand_detect_interface.so | grep "LIBRARY_VERSION"
                        # strings libhand_detect_interface.so | grep "Library Version"
                        # strings libhand_detect_interface.so | grep "Build Timestamp"