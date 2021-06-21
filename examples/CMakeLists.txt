cmake_minimum_required(VERSION 3.15)
project(cbr_drivers VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)  # TODO should be 17

find_package(OpenCV REQUIRED)


add_executable(v4l2_example
  v4l2_example.cpp
)
target_include_directories(v4l2_example
PRIVATE
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
{${PROJECT_NAME}_v4l2_INCLUDE_DIRS}
)
target_link_libraries(v4l2_example
 ${PROJECT_NAME}_v4l2
 ${OpenCV_LIBS}
)

add_executable(keyboard_example
  keyboard_example.cpp
)
target_include_directories(keyboard_example
PRIVATE
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
{${PROJECT_NAME}_keyboard_INCLUDE_DIRS}
)
target_link_libraries(keyboard_example
 ${PROJECT_NAME}_keyboard
 pthread
)

add_executable(udp_example
  udp_example.cpp
)
target_include_directories(udp_example
PRIVATE
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
{${PROJECT_NAME}_udp_INCLUDE_DIRS}
)
target_link_libraries(udp_example
 ${PROJECT_NAME}_udp
 pthread
)
