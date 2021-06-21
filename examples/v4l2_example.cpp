// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_drivers/blob/master/LICENSE

#include <cbr_drivers/v4l2_driver.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <thread>
#include <iostream>
#include <iomanip>

int main(int argc, char ** argv)
{
  if (argc != 5) {
    std::cerr << "usage: v4l2_driver_example <device> <width> <height> <fps>" << std::endl;
    exit(EXIT_FAILURE);
  }

  cbr::V4L2Driver dev(argv[1]);

  auto fmts = dev.list_formats();

  for (auto control : dev.list_controls()) {
    std::cout <<
      "Control : " << control.id << ": " << control.name <<
      ", range [" << control.ival.minimum << ", " <<
      control.ival.maximum << "]" << std::endl;
  }

  // Retrieve the available formats from the driver
  std::cout << "AVAILABLE FORMATS:" << std::endl;
  for (auto fmt : fmts) {
    std::cout << " - " << fmt.format << ": " << fmt.width << "x" << fmt.height << " @ " <<
      std::setprecision(3) << fmt.fps << std::endl;
  }

  // Set the desired format
  dev.set_format(std::stoi(argv[2]), std::stoi(argv[3]), std::stoi(argv[4]), "YUYV");
  auto fmt = dev.get_format();

  std::cout << "Current format: " << cbr::fourcc_to_str(fmt.pixelformat) << " " << fmt.width <<
    "x" << fmt.height << " @ " << std::setprecision(3) << dev.get_fps() << std::endl;

  // Specify callback that is run on new frames
  dev.set_callback(
    [](const uint8_t * yuyv, const std::chrono::nanoseconds &,
    const v4l2_pix_format & fmt) {
      cv::Mat img_yuyv(fmt.height, fmt.width, CV_8UC2, const_cast<uint8_t *>(yuyv));
      cv::Mat img_rgb;
      cv::cvtColor(img_yuyv, img_rgb, cv::COLOR_YUV2BGR_YUYV);

      cv::imshow("cam", img_rgb);
      cv::waitKey(1);
    });

  // Start capturing
  dev.start();

  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  // Stop it
  dev.stop();
}
