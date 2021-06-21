// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_drivers/blob/master/LICENSE

#ifndef CBR_DRIVERS__V4L2_DRIVER_HPP_
#define CBR_DRIVERS__V4L2_DRIVER_HPP_

#include <linux/videodev2.h>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <iostream>

namespace cbr
{

class V4L2Driver
{
public:
  /**
   * Helper structures for interfacing with the driver
   */
  struct buffer_t
  {
    void * pos;       // memory location of buffer
    std::size_t len;  // size in bytes
  };

  struct format_t
  {
    std::string format;
    uint32_t width, height;
    float fps;
  };

  enum class ctrl_type
  {
    INTEGER = V4L2_CTRL_TYPE_INTEGER,
    BOOLEAN = V4L2_CTRL_TYPE_BOOLEAN,
    MENU = V4L2_CTRL_TYPE_MENU
  };

  struct ival_t
  {
    int32_t minimum;
    int32_t maximum;
    int32_t step;
  };

  struct menu_entry_t
  {
    uint32_t index;
    std::string name;
  };

  struct ctrl_t
  {
    uint32_t id;
    ctrl_type type;
    std::string name;
    int32_t def;  // default value
    ival_t ival;  // only populated for INTEGER, BOOLEAN
    std::vector<menu_entry_t> menu;  // ony populated for MENU
  };

  /**
   * @brief v4l2 driver for capturing image streams
   *
   * This driver is flexible in terms of management: a user can supply a callback
   * that is called on frames containing new buffers, and also custom allocation and
   * de-allocation functions for allocating said buffers.
   *
   * If no allocation functions are provided malloc()/free() is used.
   *
   * @param device the v4l object (default /dev/video0)
   * @param n_buf the number of buffers to use (default 4)
   */
  explicit V4L2Driver(std::string device = "/dev/video0", uint32_t n_buf = 4);

  V4L2Driver(const V4L2Driver &) = delete;
  V4L2Driver & operator=(const V4L2Driver & other) = delete;
  V4L2Driver(V4L2Driver &&) = delete;
  V4L2Driver & operator=(V4L2Driver &&) = delete;

  ~V4L2Driver();

  /**
   * @brief Set the callback
   *
   * NOTE: the callback should return quickly or frames will be missed
   *
   * The callback should have signature
   * void(const uint8_t *, const v4l2_pix_format &)
   * and is called on every new frame.
   */
  template<typename T>
  void set_callback(T && cb)
  {
    cb_ = std::forward<T>(cb);
  }

  /**
   * @brief Set a custom allocator
   *
   * NOTE: Use only in conjuction with set_custom_deallocator
   *
   * The allocator should have signature
   * void *(std::size_t)
   *
   * The returned pointer is freed with the custom deallocator
   */
  template<typename T>
  void set_custom_allocator(T && cb)
  {
    custom_alloc_ = std::forward<T>(cb);
  }

  /**
   * @brief Set a custom deallocator
   *
   * NOTE: Use only in conjuction with set_custom_allocator
   *
   * The deallocator should have signature
   * void(void *, std::size_t)
   */
  template<typename T>
  void set_custom_deallocator(T && cb)
  {
    custom_dealloc_ = std::forward<T>(cb);
  }

  /**
   * @brief Initialize the driver and start capturing
   */
  bool start();

  /**
   * @brief Stop capturing and de-initialize the driver
   */
  void stop();

  /**
   * @brief Get current fps
   */
  float get_fps();

  /**
   * @brief List all formats supported by the device
   */
  std::vector<format_t> list_formats();

  /**
   * @brief Get active video format
   */
  v4l2_pix_format get_format();

  /**
   * @brief Request a format from v4l2
   *
   * @param width desired frame pixel width
   * @param height desired frame pixel height
   * @param fps desired fps
   * @param fourcc desired image format in fourcc format (must be string with four characters)
   */
  bool set_format(uint32_t width, uint32_t height, uint32_t fps, std::string fourcc = "YUYV");

  /**
   * @brief List controls for the device
   */
  std::vector<ctrl_t> list_controls();

  /**
   * @brief Get control value for the device
   *
   * @param id control identifier
   * @returns the value of the control
   */
  std::optional<int32_t> get_control(uint32_t id);

  /**
   * @brief Set control for the device
   *
   * @param id control identifier
   * @param value desired value for control
   *
   * @return the new value of the control (should be equal to value)
   */
  bool set_control(uint32_t id, int32_t value);

  /**
   * @brief One-way uvc write
   * @param unit uvc unit (device-specific)
   * @param control_selector uvc control selector (device-specific)
   * @param buf buffer to send over UVC (length 384 for USB3, length 64 for USB2)
   */
  bool uvc_set(uint8_t unit, uint8_t control_selector, std::vector<uint8_t> & buf);

  /**
   * @brief Two-way uvc communication (write followed by read)
   * @param unit uvc unit (device-specific)
   * @param control_selector uvc control selector (device-specific)
   * @param buf buffer to send over UVC (length 384 for USB3, length 64 for USB2)
   *
   * The result is written into buf in accordance with the device protocol
   */
  bool uvc_set_get(uint8_t unit, uint8_t control_selector, std::vector<uint8_t> & buf);

protected:
  /**
   * @brief Internal method to read the current frame format from V4L2
   */
  bool update_format();

  /**
   * @brief Internal method for capturing frames
   */
  bool capture_frame();

  /**
   * @brief Internal method that runs in the streaming thread
   */
  void streaming_fcn();

  /**
   * @brief Wrapper around system ioctl call
   */
  int xioctl(uint64_t request, void * arg);

protected:
  uint32_t n_buf_;
  std::atomic<bool> running_{false};

  std::mutex fd_mtx_;
  int fd_{0};  // file descriptor handle

  // user buffers that v4l writes into
  std::vector<buffer_t> buffers_{};

  // current device configuration
  v4l2_pix_format fmt_cur_{};
  float fps_{};

  std::thread streaming_thread_;

  std::optional<std::function<void(const uint8_t *, const std::chrono::nanoseconds &,
    const v4l2_pix_format &)>> cb_{std::nullopt};
  std::optional<std::function<void *(std::size_t)>> custom_alloc_{std::nullopt};
  std::optional<std::function<void(void *, std::size_t)>> custom_dealloc_{std::nullopt};
};

/**
 * @brief Convert fourcc code to string
 */
inline static std::string fourcc_to_str(uint32_t fourcc)
{
  std::string ret(' ', 4);
  for (uint32_t i = 0; i < 4; ++i) {
    ret[i] = ((fourcc >> (i * 8)) & 0xFF);
  }
  return ret;
}

}  // namespace cbr

#endif  // CBR_DRIVERS__V4L2_DRIVER_HPP_
