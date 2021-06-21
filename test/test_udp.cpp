// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_drivers/blob/master/LICENSE

#include <gtest/gtest.h>

#include <cbr_drivers/udp_server.hpp>

#include <vector>

TEST(UDP, selfLoopSendRecv) {
  cbr::UDPServer serv(false);
  serv.init(1000, 1560, "0.0.0.0", 1560, "0.0.0.0");
  serv.sendMsg({0u});
  sleep(1);
  ASSERT_EQ(serv.getMsg().data.size(), 1);
  ASSERT_EQ(serv.getMsg().data[0], 0u);
}

TEST(UDP, selfLoopCb) {
  cbr::UDPServer serv(false);
  serv.init(1000, 1560, "0.0.0.0", 1560, "0.0.0.0");
  std::vector<uint8_t> recv;
  auto cb = [&recv](const auto & msg) {recv = msg.data;};
  serv.subscribe(cb);
  serv.sendMsg({0u});
  sleep(1);
  ASSERT_EQ(recv.size(), 1);
  ASSERT_EQ(recv[0], 0u);
}
