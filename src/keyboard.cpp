// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_drivers/blob/master/LICENSE

#include "cbr_drivers/keyboard.hpp"

#include <sys/select.h>
#include <termios.h>

#include <stdexcept>
#include <string>
#include <memory>

namespace cbr
{

Keyboard::Keyboard(const std::string & dev)
{
  active_ = false;
  keyboard_fd_ = 0;
  keyboard_fd_ = open(dev.c_str(), O_RDONLY);  // NOLINT
  if (keyboard_fd_ < 1) {
    throw std::runtime_error("Cannot open: " + dev);
  }
  keyboard_ev_ = std::make_unique<input_event>();
  keyboard_st_ = std::make_unique<keyboard_state>();

  termios term{};
  tcgetattr(keyboard_fd_, &term);
  term.c_lflag &= ~ICANON;        // Set non-canonical mode
  term.c_cc[VTIME] = 10;        // Set timeout of 1.0 seconds
  tcsetattr(keyboard_fd_, TCSANOW, &term);

  ioctl(keyboard_fd_, EVIOCGNAME(256), name_);  // NOLINT
  active_ = true;
  thread_ = std::thread(&Keyboard::loop, this);
}

Keyboard::~Keyboard()
{
  if (keyboard_fd_ > 0) {
    active_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    close(keyboard_fd_);
  }
}

void Keyboard::loop()
{
  while (active_) {
    readEv();
  }
}

void Keyboard::readEv()
{
  int32_t bytes = read(keyboard_fd_, keyboard_ev_.get(), sizeof(*keyboard_ev_));
  if (bytes > 0) {
    if (keyboard_ev_->type & EV_KEY) {
      keyboard_st_->keys.at(keyboard_ev_->code) = keyboard_ev_->value;
    }
  }
}

int16_t Keyboard::getKeyState(int16_t key)
{
  return keyboard_st_->keys.at(key);
}

}  // namespace cbr
