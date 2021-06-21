// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_drivers/blob/master/LICENSE

#include <cbr_drivers/udp_server.hpp>

#include <thread>
#include <iostream>
#include <chrono>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  cbr::UDPServer serv1(true), serv2(true);
  serv1.init(1000, 1230, "0.0.0.0", 1231, "0.0.0.0");
  serv2.init(1000, 1231, "0.0.0.0", 1230, "0.0.0.0");

  std::string msgString("Hello world");
  std::vector<uint8_t> msg(msgString.begin(), msgString.end());

  auto subCallback1 = [](const cbr::UDPServer::UDPMsg & msg) {
      std::cout << "Server 1 received: " <<
        std::string(msg.data.begin(), msg.data.end()) << std::endl;
    };

  auto subCallback2 = [](const cbr::UDPServer::UDPMsg & msg) {
      std::cout << "Server 2 received: " <<
        std::string(msg.data.begin(), msg.data.end()) << std::endl;
    };

  serv1.subscribe(subCallback1);
  serv2.subscribe(subCallback2);

  serv1.sendMsg(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  serv2.sendMsg(serv2.getMsg().data);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}
