// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_drivers/blob/master/LICENSE

#include <cbr_drivers/v4l2_driver.hpp>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <fcntl.h>
#include <malloc.h>
#include <unistd.h>

#include <linux/usb/video.h>
#include <linux/uvcvideo.h>

#include <errno.h>

#include <filesystem>

#include <string>
#include <vector>


namespace cbr
{

V4L2Driver::V4L2Driver(std::string device, uint32_t n_buf)
: n_buf_(n_buf),
  running_(false)
{
  if (!std::filesystem::exists(device)) {
    throw std::runtime_error("Device does not exist");
  }
  fd_ = open(device.c_str(), O_RDWR | O_NONBLOCK);

  if (fcntl(fd_, F_GETFL) == -1) {
    throw std::runtime_error("File descriptor invalid");
  }
}


V4L2Driver::~V4L2Driver()
{
  if (running_) {
    stop();
  }
  close(fd_);
}


int V4L2Driver::xioctl(uint64_t request, void * arg)
{
  int r;
  do {
    std::lock_guard lock(fd_mtx_);
    r = ioctl(fd_, request, arg);
  } while (-1 == r && errno == EINTR);
  return r;
}


bool V4L2Driver::start()
{
  // query capabilites
  v4l2_capability caps{};
  memset(&caps, 0, sizeof(caps));
  if (-1 == xioctl(VIDIOC_QUERYCAP, &caps)) {
    std::cerr << "VIDIOC_QUERYCAP failed, errno=" << strerror(errno) << std::endl;
    return false;
  }

  if (!(caps.device_caps & V4L2_CAP_STREAMING)) {
    std::cerr << "Device not capable of streaming" << std::endl;
    return false;
  }

  update_format();

  // request buffers
  v4l2_requestbuffers req{};
  memset(&req, 0, sizeof(v4l2_requestbuffers));
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;
  req.count = n_buf_;
  if (-1 == xioctl(VIDIOC_REQBUFS, &req)) {
    std::cerr << "VIDIOC_REQBUFS failed, errno=" << strerror(errno) << std::endl;
    return false;
  }

  if (req.count < 2) {
    std::cerr << "Not enough buffers" << std::endl;
  } else if (req.count != n_buf_) {
    std::cerr << "Only got " << req.count << "/" << buffers_.size() << "buffers" << std::endl;
  }

  // allocate buffers in userspace
  buffers_.resize(n_buf_);
  for (std::size_t i = 0; i != n_buf_; ++i) {
    int page_size = getpagesize();
    std::size_t buffer_size = (fmt_cur_.sizeimage + page_size - 1) & ~(page_size - 1);
    if (custom_alloc_) {
      buffers_[i].pos = custom_alloc_.value()(buffer_size);
      buffers_[i].len = buffer_size;
    } else {
      buffers_[i].pos = memalign(page_size, buffer_size);
      buffers_[i].len = buffer_size;
    }
  }

  // enqueue buffers
  for (auto i = 0u; i != buffers_.size(); ++i) {
    v4l2_buffer buf;
    memset(&buf, 0, sizeof(v4l2_buffer));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_USERPTR;
    buf.index = i;
    buf.m.userptr = reinterpret_cast<intptr_t>(buffers_[i].pos);
    buf.length = buffers_[i].len;

    if (-1 == xioctl(VIDIOC_QBUF, &buf)) {
      std::cerr << "VIDIOC_QBUF failed, errno=" << strerror(errno) << std::endl;
      return false;
    }
  }

  int priority = V4L2_PRIORITY_RECORD;
  if (-1 == xioctl(VIDIOC_S_PRIORITY, &priority) ) {
    std::cerr << "VIDIOC_S_PRIORITY failed, errno=" << strerror(errno) << std::endl;
    return false;
  }

  if (-1 == xioctl(VIDIOC_G_PRIORITY, &priority) ) {
    std::cerr << "VIDIOC_G_PRIORITY failed, errno=" << strerror(errno) << std::endl;
    return false;
  }

  if (priority != V4L2_PRIORITY_RECORD) {
    std::cerr << "Could not set priority RECORD, got " << priority << std::endl;
  }

  // start capturing
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(VIDIOC_STREAMON, &type)) {
    std::cerr << "VIDIOC_STREAMON failed, errno=" << strerror(errno) << std::endl;
    return false;
  }

  running_ = true;
  streaming_thread_ = std::thread(&V4L2Driver::streaming_fcn, this);
  return true;
}


void V4L2Driver::stop()
{
  running_ = false;

  if (streaming_thread_.joinable()) {
    streaming_thread_.join();

    // stop capturing
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(VIDIOC_STREAMOFF, &type)) {
      std::cerr << "VIDIOC_STREAMOFF failed, errno=" << strerror(errno) << std::endl;
    }
  }

  // free memory
  for (auto i = 0u; i != buffers_.size(); ++i) {
    if (custom_dealloc_) {
      custom_dealloc_.value()(buffers_[i].pos, buffers_[i].len);
    } else {
      free(buffers_[i].pos);
    }
  }

  buffers_.clear();
}


std::vector<V4L2Driver::ctrl_t> V4L2Driver::list_controls()
{
  std::vector<ctrl_t> ret;

  v4l2_queryctrl queryctrl{};
  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
  while (-1 != (xioctl(VIDIOC_QUERYCTRL, &queryctrl))) {
    // Ignore disabled controls
    if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
      continue;
    }

    ctrl_t ctrl{};
    ctrl.id = queryctrl.id;
    ctrl.name = std::string(reinterpret_cast<char *>(queryctrl.name));
    ctrl.def = queryctrl.default_value;
    ctrl.ival = ival_t{queryctrl.minimum, queryctrl.maximum, queryctrl.step};

    if (queryctrl.type == V4L2_CTRL_TYPE_BOOLEAN || queryctrl.type == V4L2_CTRL_TYPE_INTEGER) {
      ctrl.type = static_cast<ctrl_type>(queryctrl.type);
    } else if (queryctrl.type == V4L2_CTRL_TYPE_MENU) {
      ctrl.type = ctrl_type::MENU;
      v4l2_querymenu querymenu{};
      querymenu.id = queryctrl.id;
      // Query all enum values
      for (auto i = queryctrl.minimum; i <= queryctrl.maximum; i++) {
        querymenu.index = i;
        if (-1 != (xioctl(VIDIOC_QUERYMENU, &querymenu))) {
          ctrl.menu.push_back(
            menu_entry_t{querymenu.index,
              reinterpret_cast<char *>(querymenu.name)});
        }
      }
      // Get ready to query next item
    }
    ret.push_back(ctrl);
    queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
  }

  return ret;
}


std::optional<int32_t> V4L2Driver::get_control(uint32_t id)
{
  v4l2_control control;
  memset(&control, 0, sizeof(control));

  control.id = id;
  if (-1 == xioctl(VIDIOC_G_CTRL, &control)) {
    std::cerr << "VIDIOC_G_CTRL failed, errno=" << strerror(errno) << std::endl;
    return {};
  }

  return control.value;
}


bool V4L2Driver::set_control(uint32_t id, int32_t value)
{
  v4l2_control control;
  memset(&control, 0, sizeof(control));

  control.id = id;
  control.value = value;

  if (-1 == xioctl(VIDIOC_S_CTRL, &control)) {
    std::cerr << "VIDIOC_S_CTRL failed, errno=" << strerror(errno) << std::endl;
  }

  auto new_val = get_control(id);
  if (new_val && new_val.value() != value) {
    std::cerr << "Could not set " << id << " to " << value << std::endl;
    return false;
  }

  return true;
}


bool V4L2Driver::uvc_set(
  uint8_t unit, uint8_t control_selector,
  std::vector<uint8_t> & buf)
{
  std::array<uint8_t, 2> tmp{0, 0};

  // check the expected length
  uvc_xu_control_query xu_query_info{};
  xu_query_info.unit = unit;
  xu_query_info.selector = control_selector;
  xu_query_info.query = UVC_GET_LEN;
  xu_query_info.size = 2;
  xu_query_info.data = tmp.data();

  // lock during _sequence_ of calls, must use ioctl since xioctl locks same mtx
  std::lock_guard lock(fd_mtx_);

  if (-1 == ioctl(fd_, UVCIOC_CTRL_QUERY, &xu_query_info)) {
    std::cerr << "UVCIOC_CTRL_QUERY failed, errno=" << strerror(errno) << std::endl;
    return false;
  }

  uint32_t len = (xu_query_info.data[1] << 8) + xu_query_info.data[0];

  if (len > buf.size()) {
    std::cerr << "uvc_set: buffer too small, got " <<
      buf.size() << " expected " << len << std::endl;
    return false;
  }

  // check ranges
  xu_query_info.query = UVC_GET_MIN;

  // set value
  uvc_xu_control_query xu_query_send{};
  xu_query_send.unit = unit;
  xu_query_send.selector = control_selector;
  xu_query_send.query = UVC_SET_CUR;
  xu_query_send.size = static_cast<uint16_t>(len);
  xu_query_send.data = buf.data();

  if (-1 == ioctl(fd_, UVCIOC_CTRL_QUERY, &xu_query_send)) {
    std::cerr << "UVCIOC_CTRL_QUERY failed, errno=" << strerror(errno) << std::endl;
    return false;
  }

  usleep(2000);

  return true;
}


bool V4L2Driver::uvc_set_get(uint8_t unit, uint8_t control_selector, std::vector<uint8_t> & buf)
{
  std::array<uint8_t, 2> tmp{0, 0};

  // check the expected length
  uvc_xu_control_query xu_query_info{};
  xu_query_info.unit = unit;
  xu_query_info.selector = control_selector;
  xu_query_info.query = UVC_GET_LEN;
  xu_query_info.size = 2;
  xu_query_info.data = tmp.data();

  // lock during _sequence_ of calls, must use ioctl since xioctl locks same mtx
  std::lock_guard lock(fd_mtx_);

  if (-1 == ioctl(fd_, UVCIOC_CTRL_QUERY, &xu_query_info)) {
    std::cerr << "UVCIOC_CTRL_QUERY failed, errno=" << strerror(errno) << std::endl;
    return false;
  }

  uint32_t len = (xu_query_info.data[1] << 8) + xu_query_info.data[0];

  if (len > buf.size()) {
    std::cerr << "uvc_set: buffer too small, got " <<
      buf.size() << " expected " << len << std::endl;
    return false;
  }

  // set value
  uvc_xu_control_query xu_query_send{};
  xu_query_send.unit = unit;
  xu_query_send.selector = control_selector;
  xu_query_send.query = UVC_SET_CUR;
  xu_query_send.size = static_cast<uint16_t>(len);
  xu_query_send.data = buf.data();

  if (-1 == ioctl(fd_, UVCIOC_CTRL_QUERY, &xu_query_send)) {
    std::cerr << "UVCIOC_CTRL_QUERY failed, errno=" << strerror(errno) << std::endl;
    return false;
  }

  usleep(2000);

  uvc_xu_control_query xu_query_get{};
  xu_query_get.unit = unit;
  xu_query_get.selector = control_selector;
  xu_query_get.query = UVC_GET_CUR;
  xu_query_get.size = static_cast<uint16_t>(len);
  xu_query_get.data = buf.data();

  if (-1 == ioctl(fd_, UVCIOC_CTRL_QUERY, &xu_query_get)) {
    std::cerr << "UVCIOC_CTRL_QUERY failed, errno=" << strerror(errno) << std::endl;
    return false;
  }

  return true;
}


bool V4L2Driver::update_format()
{
  // read format
  v4l2_format fmtCur{};
  memset(&fmtCur, 0, sizeof(fmtCur));
  fmtCur.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(VIDIOC_G_FMT, &fmtCur)) {
    std::cerr << "VIDIOC_G_FMT failed, errno=" << strerror(errno) << std::endl;
    return false;
  }
  fmt_cur_ = fmtCur.fmt.pix;


  // read fps
  v4l2_streamparm streamparm{};
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (-1 == xioctl(VIDIOC_G_PARM, &streamparm)) {
    std::cerr << "VIDIOC_G_PARM failed, errno=" << strerror(errno) << std::endl;
    return false;
  }
  fps_ = 1.f * streamparm.parm.capture.timeperframe.denominator /
    streamparm.parm.capture.timeperframe.numerator;

  return true;
}


bool V4L2Driver::set_format(uint32_t width, uint32_t height, uint32_t fps, std::string fourcc)
{
  if (fourcc.size() != 4) {
    std::cerr << "fourcc must be of length four" << std::endl;
    return false;
  }

  v4l2_format fmtReq{};
  memset(&fmtReq, 0, sizeof(fmtReq));

  fmtReq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmtReq.fmt.pix.pixelformat = v4l2_fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);
  fmtReq.fmt.pix.field = V4L2_FIELD_ANY;
  fmtReq.fmt.pix.width = width;
  fmtReq.fmt.pix.height = height;

  if (-1 == xioctl(VIDIOC_S_FMT, &fmtReq)) {
    std::cerr << "VIDIOC_S_FMT failed: errno=" << strerror(errno) << std::endl;
    return false;
  }

  // set fps
  v4l2_streamparm streamparm{};
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (-1 == xioctl(VIDIOC_G_PARM, &streamparm)) {
    std::cerr << "VIDIOC_G_PARM failed, errno=" << strerror(errno) << std::endl;
    return false;
  }

  streamparm.parm.capture.timeperframe.numerator = 1;
  streamparm.parm.capture.timeperframe.denominator = fps;

  if (-1 == xioctl(VIDIOC_S_PARM, &streamparm)) {
    std::cerr << "VIDIOC_S_PARM failed, errno=" << strerror(errno) << std::endl;
    return false;
  }

  // read new format and fps
  update_format();

  return true;
}


v4l2_pix_format V4L2Driver::get_format()
{
  return fmt_cur_;
}


float V4L2Driver::get_fps()
{
  return fps_;
}


bool V4L2Driver::capture_frame()
{
  v4l2_buffer cap;
  memset(&cap, 0, sizeof(cap));

  cap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  cap.memory = V4L2_MEMORY_USERPTR;

  if (-1 == xioctl(VIDIOC_DQBUF, &cap)) {
    switch (errno) {
      case EAGAIN:
        return 0;   // no frame
        break;

      case EIO:   // can ignore
        break;

      default:
        std::cerr << "DQBUF error: errno=" << strerror(errno) << std::endl;
        return 0;
        break;
    }
  }

  assert(cap.index < buffers_.size());
  assert(cap.length <= buffers_[cap.index].len);
  assert(cap.bytesused <= buffers_[cap.index].len);

  if (cb_) {
    std::chrono::nanoseconds ts = std::chrono::seconds(cap.timestamp.tv_sec) +
      std::chrono::microseconds(cap.timestamp.tv_usec);
    cb_.value()(reinterpret_cast<const uint8_t *>(buffers_[cap.index].pos), ts, fmt_cur_);
  }

  // re-queue buffer
  if (-1 == xioctl(VIDIOC_QBUF, &cap)) {
    std::cerr << "QBUF error: errno=" << strerror(errno) << std::endl;
  }

  return true;   // got a frame
}


void V4L2Driver::streaming_fcn()
{
  while (running_) {
    fd_set fds;
    timeval tv{1, 0};   // 1s timeout

    FD_ZERO(&fds);
    FD_SET(fd_, &fds);

    int r = select(fd_ + 1, &fds, NULL, NULL, &tv);

    if (-1 == r && errno != EINTR) {
      std::cerr << "Select fail: " << strerror(errno) << std::endl;
      continue;
    } else if (0 == r) {
      std::cerr << "Select timeout: " << strerror(errno) << std::endl;
      continue;
    }

    capture_frame();
  }
}


std::vector<V4L2Driver::format_t> V4L2Driver::list_formats()
{
  std::vector<V4L2Driver::format_t> ret;

  v4l2_fmtdesc fmtDsc{};      // format
  fmtDsc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2_frmsizeenum fsDsc{};   // size
  v4l2_frmivalenum fiDsc{};   // fps
  for (fmtDsc.index = 0; -1 != xioctl(VIDIOC_ENUM_FMT, &fmtDsc); ++fmtDsc.index) {
    fsDsc.pixel_format = fmtDsc.pixelformat;
    for (fsDsc.index = 0; -1 != xioctl(VIDIOC_ENUM_FRAMESIZES, &fsDsc); ++fsDsc.index) {
      if (fsDsc.type != V4L2_FRMSIZE_TYPE_DISCRETE) {
        std::cerr << "Unsupported size type " << fsDsc.type << std::endl;
        continue;
      }

      fiDsc.pixel_format = fmtDsc.pixelformat;
      fiDsc.width = fsDsc.discrete.width;
      fiDsc.height = fsDsc.discrete.height;
      for (fiDsc.index = 0; -1 != xioctl(VIDIOC_ENUM_FRAMEINTERVALS, &fiDsc);
        ++fiDsc.index)
      {
        if (fiDsc.type != V4L2_FRMIVAL_TYPE_DISCRETE) {
          std::cerr << "Unsupported frame interval type " << fiDsc.type << std::endl;
          continue;
        }
        ret.push_back(
          {
            fourcc_to_str(fmtDsc.pixelformat), fsDsc.discrete.width, fsDsc.discrete.height,
            1.f * fiDsc.discrete.denominator / fiDsc.discrete.numerator
          });
      }
    }
  }

  return ret;
}

}  // namespace cbr
