// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_drivers/blob/master/LICENSE

#include <gtest/gtest.h>

#include <cbr_drivers/v4l2_driver.hpp>

#include <cstdio>
#include <string>

TEST(V4L2, invalidDevice) {
  std::string fn = std::tmpnam(nullptr);
  try {
    cbr::V4L2Driver v4l2(fn);
    FAIL() << "Incorrect behavior, successfull construction on invalid device";
  } catch (std::runtime_error const & err) {
    EXPECT_EQ(err.what(), std::string("Device does not exist"));
  } catch (...) {
    FAIL() << "Incorrect error type, expected std::runtime_error";
  }
}
