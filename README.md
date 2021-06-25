# ```cbr_drivers```

[![build_and_test](https://github.com/yamaha-bps/cbr_drivers/actions/workflows/build_and_test.yaml/badge.svg)](https://github.com/yamaha-bps/cbr_drivers/actions/workflows/build_and_test.yaml)

## Overview
A bundle of useful driver tools for linux devices.  

### V4L2
A V4L2 driver for capturing image streams. A user can supply a callback that is called on frames containing new buffers, as well as custom allocation and de-allocation functions for those buffers.

### UDP
A UDP server that allows a publisher-subscriber relationship with another device over UDP.

### Keyboard
A keyboard device interface for lightweight and simple key state detection.

## Build and Installation
```bash
git clone https://github.com/yamaha-bps/cbr_drivers.git
mkdir cbr_drivers/build && cd cbr_drivers/build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
sudo make install
```
Note: Each tool is compiled as a separate shared library.

## Usage
Find the project and link the library you intend to use.
```cmake
find_package(cbr_drivers)
```
```cmake
target_link_libraries(target
  cbr_drivers::cbr_drivers_v4l2
  cbr_drivers::cbr_drivers_keyboard
  cbr_drivers::cbr_drivers_udp
)
```
[Reference](https://yamaha-bps.github.io/cbr_drivers/)


## Examples
- [V4L2](examples/v4l2_example.cpp)
- [Keyboard](examples/keyboard_example.cpp)
- [UDP](examples/udp_example.cpp)


