// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_drivers/blob/master/LICENSE

#ifndef CBR_DRIVERS__UDP_SERVER_HPP_
#define CBR_DRIVERS__UDP_SERVER_HPP_

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cerrno>

#include <cstdint>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>
#include <cstring>

#include <chrono>
#include <thread>
#include <mutex>
#include <random>
#include <utility>
#include <functional>
#include <atomic>

namespace cbr
{

class UDPServer
{
public:
  using clock_t = std::chrono::high_resolution_clock;
  using timePoint_t = std::chrono::time_point<clock_t>;

  /**
   * @brief Message struct with data and metadata (seq id and timestamp)
   *
   */
  struct UDPMsg
  {
    size_t seq;
    timePoint_t time;
    std::vector<uint8_t> data;
  };

  /**
   * @brief UDP server
   *
   * This server is particularly useful for creating communication a bridge with another
   * device over UDP in a publisher-subscriber manner.
   *
   * @param verbose If true, prints additional server state notifications
   */

  explicit UDPServer(bool verbose);
  UDPServer() = default;
  UDPServer(const UDPServer & rhs) = delete;
  UDPServer(UDPServer && rhs) = delete;
  UDPServer & operator=(const UDPServer & rhs) = delete;
  UDPServer & operator=(UDPServer && rhs) = delete;
  ~UDPServer();

  /**
   * @brief Start server. Will restart server if already running.
   *
   * @param recvMaxSize Size for message receipt
   * @param sendPort Default destination port to for outgoing messages
   * @param sendAddr Default destination address for outgoing messsages
   * @param recvPort Bound port for incoming messages
   * @param recvAddr Address used to filter incoming messages
   */
  void init(
    uint32_t recvMaxSize,
    uint16_t sendPort = 1560,
    std::string_view sendAddr = "",
    uint16_t recvPort = 1561,
    std::string_view recvAddr = "");

  /**
   * @brief Initialization with random ports and addresses
   *
   */
  void init();

  /**
   * @brief Fetch last received message with metadata
   */
  UDPMsg getMsg();

  /**
   * @brief Send message to default destination
   *
   * @param msg Message data
   */
  void sendMsg(const std::vector<uint8_t> & msg) const;

  /**
   * @brief Send message to selected destination
   *
   * @param msg Message data
   * @param sendPort Destination port
   * @param sendAddr Destination address
   */
  void sendMsg(
    const std::vector<uint8_t> & msg,
    uint16_t sendPort,
    const std::string & sendAddr) const;

  /**
   * @brief Subscribe a callback function to the server that is called upon message receipt
   *
   * @param cb Callback function for received messages
   */
  template<typename T>
  void subscribe(T && cb)
  {
    if (verbose_) {
      std::cout << "Subscribed to UDPServer." << std::endl;
    }
    cb_ = std::forward<T>(cb);
    subscribed_ = true;
  }

private:
  void udpReceiveThreadFun();

  bool initialized_ = false;
  bool verbose_ = false;
  std::atomic<bool> isRunning_ = false;
  bool subscribed_ = false;

  std::size_t seq_ = 0;
  uint32_t recvMsgSize_ = 0;
  std::vector<uint8_t> recvBuff_{};
  timePoint_t recvTimePoint_{};
  uint16_t sendPort_{};
  std::string sendAddr_{};
  uint16_t recvPort_{};
  std::string recvAddr_{};

  int32_t socket_recv_{};
  int32_t socket_send_{};
  sockaddr_in servaddr_recv_ {};
  sockaddr_in servaddr_send_ {};

  std::thread recvThread_{};
  std::mutex recvMutex_{};

  UDPMsg currMsg_{};

  std::function<void(const UDPMsg & /*msg*/)> cb_;
};

}  // namespace cbr
#endif  // CBR_DRIVERS__UDP_SERVER_HPP_
