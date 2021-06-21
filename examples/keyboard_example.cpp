// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_drivers/blob/master/LICENSE

#include <cbr_drivers/keyboard.hpp>

#include <thread>
#include <chrono>
#include <iostream>

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: keyboard_example <device>" << std::endl;
    exit(EXIT_FAILURE);
  }
  cbr::Keyboard keyboard(argv[1]);

  for (auto i = 0u; i < 500; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::cout << std::endl << "W: " << keyboard.getKeyState(KEY_W) << " A: " <<
      keyboard.getKeyState(KEY_A) <<
      " S: " << keyboard.getKeyState(KEY_S) << " D: " << keyboard.getKeyState(KEY_D) << std::endl;
  }
}
