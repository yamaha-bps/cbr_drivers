cmake_minimum_required(VERSION 3.15)
project(cbr_drivers VERSION 0.1.0)

include(CTest)
include(GoogleTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_EXAMPLES "Build examples." OFF)

# ---------------------------------------------------------------------------------------
# TARGETS
# ---------------------------------------------------------------------------------------

add_library(${PROJECT_NAME}_v4l2 SHARED
  src/v4l2_driver.cpp
)
target_include_directories(${PROJECT_NAME}_v4l2
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_v4l2
PUBLIC
  hidapi-hidraw
)
target_compile_options(${PROJECT_NAME}_v4l2 PRIVATE -Werror -Wextra -Wpedantic)

add_library(${PROJECT_NAME}_keyboard SHARED
  src/keyboard.cpp
)
target_include_directories(${PROJECT_NAME}_keyboard
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(${PROJECT_NAME}_keyboard PRIVATE -Werror -Wextra -Wpedantic)

add_library(${PROJECT_NAME}_udp SHARED
  src/udp_server.cpp
)
target_include_directories(${PROJECT_NAME}_udp
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(${PROJECT_NAME}_udp PRIVATE -Werror -Wextra -Wpedantic)

# ---------------------------------------------------------------------------------------
# EXAMPLES
# ---------------------------------------------------------------------------------------

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# ---------------------------------------------------------------------------------------
# INSTALLATION
# ---------------------------------------------------------------------------------------

include(GNUInstallDirs)

# Header files
install(
  DIRECTORY    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  DESTINATION  ${CMAKE_INSTALL_INCLUDEDIR}
)

# Targets
install(
  TARGETS               ${PROJECT_NAME}_v4l2
                        ${PROJECT_NAME}_udp
                        ${PROJECT_NAME}_keyboard
  EXPORT                ${PROJECT_NAME}_targets
  LIBRARY DESTINATION   ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION   ${CMAKE_INSTALL_LIBDIR}
)

# ---------------------------------------------------------------------------------------
# CMAKE CONFIGURATION
# ---------------------------------------------------------------------------------------

set(export_dest_dir ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

# Generate TARGETS files
install(
  EXPORT       ${PROJECT_NAME}_targets
  FILE         ${PROJECT_NAME}Targets.cmake
  NAMESPACE    ${PROJECT_NAME}::
  DESTINATION  ${export_dest_dir}
)

# Generate CONFIG file
include(CMakePackageConfigHelpers)
configure_package_config_file(
  cmake/ConfigTemplate.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${export_dest_dir}
)

# Generate VERSION file
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Version.cmake
  VERSION ${PACKAGE_VERSION}
  COMPATIBILITY SameMajorVersion
)

# Install CONFIG and VERSION files
install(
  FILES        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
               ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Version.cmake
  DESTINATION  ${export_dest_dir}
)

# Add target for uninstallation
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/uninstall.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake
  IMMEDIATE @ONLY
)
add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake)

# ---------------------------------------------------------------------------------------
# TESTING
# ---------------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(GTest REQUIRED)

  add_executable(${PROJECT_NAME}_test_v4l2 test/test_v4l2.cpp)
  target_compile_options(${PROJECT_NAME}_test_v4l2 PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(${PROJECT_NAME}_test_v4l2 PRIVATE ${PROJECT_NAME}_v4l2 GTest::Main)
  gtest_discover_tests(${PROJECT_NAME}_test_v4l2)

  add_executable(${PROJECT_NAME}_test_keyboard test/test_keyboard.cpp)
  target_compile_options(${PROJECT_NAME}_test_keyboard PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(${PROJECT_NAME}_test_keyboard PRIVATE ${PROJECT_NAME}_keyboard GTest::Main)
  gtest_discover_tests(${PROJECT_NAME}_test_keyboard)

  add_executable(${PROJECT_NAME}_test_udp test/test_udp.cpp)
  target_compile_options(${PROJECT_NAME}_test_udp PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(${PROJECT_NAME}_test_udp PRIVATE ${PROJECT_NAME}_udp GTest::Main)
  gtest_discover_tests(${PROJECT_NAME}_test_udp)
endif()
