// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_drivers/blob/master/LICENSE

#ifndef CBR_DRIVERS__KEYBOARD_HPP_
#define CBR_DRIVERS__KEYBOARD_HPP_

#include <unistd.h>
#include <fcntl.h>
#include <linux/input.h>

#include <thread>
#include <string>
#include <atomic>
#include <array>
#include <memory>

namespace cbr
{

class Keyboard
{
protected:
  struct keyboard_state
  {
    std::array<int16_t, KEY_CNT> keys{};
  };

public:
  /**
   * @brief Keyboard driver wrapper for monitoring key state changes
   *
   * @param dev Device file, e.g. /dev/input/event2
   */
  explicit Keyboard(const std::string & dev);

  Keyboard() = default;
  Keyboard(const Keyboard &) = delete;
  Keyboard(Keyboard &&) = delete;
  Keyboard & operator=(const Keyboard &) = delete;
  Keyboard & operator=(Keyboard &&) = delete;
  ~Keyboard();

  /**
   * @brief Sample the current state of a given key
   *
   * @param key Key ID to check, e.g. KEY_W
   * @return int16_t State of key: 0 for unpressed, 1 for pressed, 2 for held
   */
  int16_t getKeyState(int16_t key);

protected:
  void loop();
  void readEv();

protected:
  std::thread thread_{};
  std::atomic<bool> active_{};
  int32_t keyboard_fd_{};
  std::unique_ptr<input_event> keyboard_ev_{};
  std::unique_ptr<keyboard_state> keyboard_st_{};
  std::array<char, 256> name_{};
};

}  // namespace cbr

#endif  // CBR_DRIVERS__KEYBOARD_HPP_
