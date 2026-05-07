cmake_minimum_required(VERSION 3.16)
project(rplidar_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

set(RPLIDAR_SDK_PATH "${PROJECT_SOURCE_DIR}/sdk")

if(APPLE)
  add_compile_definitions(_MACOS)
  file(GLOB RPLIDAR_SDK_SRC
    "${RPLIDAR_SDK_PATH}/src/arch/macOS/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/*.cpp"
  )
else()
  file(GLOB RPLIDAR_SDK_SRC
    "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/*.cpp"
  )
endif()

add_library(slamtec_lidar_sdk STATIC ${RPLIDAR_SDK_SRC})
target_include_directories(slamtec_lidar_sdk
  SYSTEM
  PUBLIC
    "${RPLIDAR_SDK_PATH}/include"
    "${RPLIDAR_SDK_PATH}/src"
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(slamtec_lidar_sdk PRIVATE -w)
endif()

add_executable(rplidar_c1_node src/rplidar_c1_node.cpp)
target_link_libraries(rplidar_c1_node slamtec_lidar_sdk Threads::Threads)
ament_target_dependencies(rplidar_c1_node
  rclcpp
  sensor_msgs
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(rplidar_c1_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)

install(FILES udev/99-rplidar-c1.rules
  DESTINATION share/${PROJECT_NAME}/udev
)

install(TARGETS rplidar_c1_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
