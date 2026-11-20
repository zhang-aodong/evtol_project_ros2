cmake_minimum_required(VERSION 3.15)

project(evtol_offline_fusion LANGUAGES CXX)
option(OFFLINE_FUSION_BUILD_IMU_TEST "Build IMU covariance regression test" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(OFFLINE_FUSION_USE_CERES "Use Ceres for nonlinear factor-graph optimization" ON)
option(OFFLINE_FUSION_USE_OPENCV "Use OpenCV for sparse visual feature tracking" ON)

add_executable(offline_fusion
    app/offline_fusion_main.cpp
    src/dataset.cpp
    src/fusion.cpp
    src/initialization.cpp
)

target_include_directories(offline_fusion PRIVATE include)

if (OFFLINE_FUSION_USE_CERES)
    find_package(Ceres CONFIG REQUIRED)
    target_sources(offline_fusion PRIVATE src/ceres_fusion.cpp)
    target_link_libraries(offline_fusion PRIVATE Ceres::ceres)
    target_compile_definitions(offline_fusion PRIVATE OFFLINE_FUSION_HAS_CERES=1)
endif()

if (OFFLINE_FUSION_USE_OPENCV)
    find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs video calib3d flann)
    target_sources(offline_fusion PRIVATE src/vision.cpp src/lidar.cpp)
    target_link_libraries(offline_fusion PRIVATE ${OpenCV_LIBS})
    target_compile_definitions(offline_fusion PRIVATE
        OFFLINE_FUSION_HAS_OPENCV=1
        OFFLINE_FUSION_HAS_LIDAR=1
    )
endif()

if (MSVC)
    target_compile_options(offline_fusion PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(offline_fusion PRIVATE -Wall -Wextra -Wpedantic)
endif()

if (OFFLINE_FUSION_BUILD_IMU_TEST)
    enable_testing()
    add_executable(test_imu tests/test_imu.cpp)
    target_include_directories(test_imu PRIVATE include ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_options(test_imu PRIVATE -O1 -ffunction-sections -fdata-sections)
    target_link_options(test_imu PRIVATE -Wl,--gc-sections)
    target_link_libraries(test_imu PRIVATE Ceres::ceres)
    add_test(NAME imu_covariance COMMAND test_imu)
endif()

enable_testing()
add_executable(test_camera tests/test_camera_backend.cpp tests/test_camera_frontend.cpp)
target_include_directories(test_camera PRIVATE include)
target_compile_options(test_camera PRIVATE -O1 -UNDEBUG -ffunction-sections -fdata-sections)
target_compile_definitions(test_camera PRIVATE OFFLINE_FUSION_HAS_OPENCV=1)
target_link_options(test_camera PRIVATE -Wl,--gc-sections)
target_link_libraries(test_camera PRIVATE Ceres::ceres ${OpenCV_LIBS})
add_test(NAME camera_pitch COMMAND test_camera)

add_executable(test_lidar_geometry tests/test_lidar_geometry.cpp)
target_include_directories(test_lidar_geometry PRIVATE include)
target_compile_options(test_lidar_geometry PRIVATE -O1 -UNDEBUG -ffunction-sections -fdata-sections)
target_link_options(test_lidar_geometry PRIVATE -Wl,--gc-sections)
target_link_libraries(test_lidar_geometry PRIVATE ${OpenCV_LIBS})
add_test(NAME lidar_geometry COMMAND test_lidar_geometry)

