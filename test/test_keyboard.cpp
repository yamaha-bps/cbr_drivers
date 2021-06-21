// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_drivers/blob/master/LICENSE

#include <cbr_drivers/keyboard.hpp>

#include <gtest/gtest.h>

#include <cstdio>
#include <string>

TEST(Keyboard, invalidDevice) {
  std::string fn = std::tmpnam(nullptr);
  try {
    cbr::Keyboard keyboard(fn);
    FAIL() << "Incorrect behavior, successfull construction on invalid device";
  } catch (std::runtime_error const & err) {
    EXPECT_EQ(err.what(), std::string("Cannot open: ") + fn);
  } catch (...) {
    FAIL() << "Incorrect error type, expected std::runtime_error";
  }
}
