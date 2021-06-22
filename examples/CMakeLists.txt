find_package(OpenCV QUIET)

if(${OpenCV_FOUND})
  add_executable(v4l2_example
    v4l2_example.cpp
  )
  target_link_libraries(v4l2_example
    ${PROJECT_NAME}_v4l2
    ${OpenCV_LIBS}
  )
else()
  message(WARNING "OpenCV required for v4l2 example")
endif()

add_executable(keyboard_example
  keyboard_example.cpp
)
target_link_libraries(keyboard_example
 ${PROJECT_NAME}_keyboard
 pthread
)

add_executable(udp_example
  udp_example.cpp
)
target_link_libraries(udp_example
  ${PROJECT_NAME}_udp
  pthread
)
