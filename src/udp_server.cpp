// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_drivers/blob/master/LICENSE

#include "cbr_drivers/udp_server.hpp"

#include <vector>
#include <string>

namespace cbr
{

UDPServer::UDPServer(bool verbose)
: verbose_{verbose}
{}

UDPServer::~UDPServer()
{
  if (verbose_) {
    std::cout << "UDPServer shutting down" << std::endl;
  }

  isRunning_ = false;
  if (recvThread_.joinable()) {
    recvThread_.join();
  }

  close(socket_send_);
  close(socket_recv_);
}

void UDPServer::init()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint16_t> dis;
  init(1024, dis(gen), "", dis(gen), "");
}

void UDPServer::init(
  const uint32_t recvMaxSize,
  const uint16_t sendPort,
  const std::string_view sendAddr,
  const uint16_t recvPort,
  const std::string_view recvAddr)
{
  // Reset everything if already initialized
  if (initialized_) {
    if (verbose_) {
      std::cout << "UDPServer already initialized. Resetting server..." << std::endl;
    }
    isRunning_ = false;
    recvThread_.join();

    close(socket_send_);
    close(socket_recv_);
  } else {
    if (verbose_) {
      std::cout << "UDPServer initializing..." << std::endl;
    }
  }

  sendPort_ = sendPort;
  sendAddr_ = sendAddr;
  recvPort_ = recvPort;
  recvAddr_ = recvAddr;

  std::unique_lock lck(recvMutex_);
  recvBuff_.resize(recvMaxSize);
  recvBuff_.shrink_to_fit();
  lck.unlock();

  // Creating socket for sending
  if (verbose_) {
    if (sendAddr_.empty()) {
      std::cout << "Creating sending socket broadcasting on port: " << sendPort_ << std::endl;
    } else {
      std::cout << "Creating sending socket for: " << sendAddr_ << ":" << sendPort_ << std::endl;
    }
  }
  socket_send_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_send_ < 0) {
    throw std::runtime_error(std::string("Send socket creation failed: ") + strerror(errno));
  }
  memset(&servaddr_send_, 0, sizeof(servaddr_send_));
  servaddr_send_.sin_family = AF_INET;
  servaddr_send_.sin_port = htons(sendPort_);
  if (sendAddr_.empty()) {
    servaddr_send_.sin_addr.s_addr = INADDR_BROADCAST;
    int32_t enabled = 1;
    setsockopt(socket_send_, SOL_SOCKET, SO_BROADCAST, &enabled, sizeof(enabled));
  } else {
    servaddr_send_.sin_addr.s_addr = inet_addr(sendAddr_.c_str());
  }

  // Creating socket for receiving
  socket_recv_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (verbose_) {
    if (recvAddr_.empty()) {
      std::cout << "Creating receiving socket for port: " << recvPort_ << std::endl;
    } else {
      std::cout << "Creating receiving socket for: " << recvAddr_ << ":" << recvPort_ << std::endl;
    }
  }
  if (socket_recv_ < 0) {
    throw std::runtime_error(std::string("Receive socket creation failed: ") + strerror(errno));
  }

  memset(&servaddr_recv_, 0, sizeof(servaddr_recv_));
  servaddr_recv_.sin_family = AF_INET;
  servaddr_recv_.sin_port = htons(recvPort_);
  servaddr_recv_.sin_addr.s_addr = INADDR_ANY;

  struct timeval tv {0, 100000};

  int32_t optval = 1;
  if (setsockopt(
      socket_recv_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&tv),
      sizeof(struct timeval)) < 0)
  {
    throw std::runtime_error(
            std::string("Cannot set socket option SO_RCVTIMEO, error code: ") +
            strerror(errno));
  }
  if (setsockopt(socket_recv_, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval)) < 0) {
    throw std::runtime_error(
            std::string("Cannot set socket option SO_REUSEPORT, error code: ") +
            strerror(errno));
  }
  if (setsockopt(socket_recv_, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0) {
    throw std::runtime_error(
            std::string("Cannot set socket option SO_REUSEADDR, error code: ") +
            strerror(errno));
  }
  if (bind(
      socket_recv_, reinterpret_cast<const struct sockaddr *>(&servaddr_recv_),
      sizeof(servaddr_recv_)) < 0)
  {
    throw std::runtime_error(
            std::string("Receive socket bind failed, error code: ") +
            strerror(errno));
  }

  isRunning_ = true;
  initialized_ = true;

  if (verbose_) {
    std::cout << "Starting receiving thread..." << std::endl;
  }

  recvThread_ = std::thread(
    [this] {
      udpReceiveThreadFun();
    });

  if (verbose_) {
    std::cout << "UDPServer initialized." << std::endl;
  }
}

UDPServer::UDPMsg UDPServer::getMsg()
{
  std::lock_guard lck(recvMutex_);
  return currMsg_;
}

void UDPServer::sendMsg(const std::vector<uint8_t> & msg) const
{
  sendto(
    socket_send_,
    msg.data(),
    msg.size(),
    MSG_DONTWAIT, reinterpret_cast<const struct sockaddr *>(&servaddr_send_),
    sizeof(servaddr_send_));
}

void UDPServer::sendMsg(
  const std::vector<uint8_t> & msg,
  const uint16_t sendPort,
  const std::string & sendAddr) const
{
  sockaddr_in servaddr_send{};
  memset(&servaddr_send, 0, sizeof(servaddr_send));
  servaddr_send.sin_family = AF_INET;
  servaddr_send.sin_port = htons(sendPort);
  if (sendAddr.empty()) {
    servaddr_send.sin_addr.s_addr = INADDR_BROADCAST;
  } else {
    servaddr_send.sin_addr.s_addr = inet_addr(sendAddr.c_str());
  }

  sendto(
    socket_send_, msg.data(),
    msg.size(), MSG_DONTWAIT, (const struct sockaddr *)&servaddr_send, sizeof(servaddr_send));
}

void UDPServer::udpReceiveThreadFun()
{
  std::vector<uint8_t> recvBuffTemp(recvBuff_.size());
  sockaddr_in servaddr_recv{};
  memset(&servaddr_recv, 0, sizeof(servaddr_recv));
  socklen_t len;

  bool recvFilter = true;
  if (recvAddr_.empty()) {
    recvFilter = false;
  }

  while (isRunning_) {
    const auto n =
      recvfrom(
      socket_recv_, recvBuffTemp.data(), recvBuff_.size(), MSG_WAITALL,
      (struct sockaddr *)&servaddr_recv, &len);
    if (n > 0) {
      if (!recvFilter ||
        (recvFilter && inet_addr(recvAddr_.c_str()) == servaddr_recv.sin_addr.s_addr))
      {
        std::lock_guard lck(recvMutex_);
        recvMsgSize_ = n;
        seq_++;
        recvBuff_.assign(recvBuffTemp.begin(), recvBuffTemp.begin() + n);

        currMsg_ = {seq_,
          std::chrono::high_resolution_clock::now(),
          std::vector<uint8_t>(
            recvBuff_.begin(),
            recvBuff_.begin() + recvMsgSize_)};
        if (subscribed_) {
          cb_(currMsg_);
        }
      }
    }
  }
}

}  // namespace cbr
