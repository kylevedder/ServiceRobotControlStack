#pragma once

#include <unistd.h>
#include <iostream>

namespace util {

void PrintCurrentWorkingDirectory() {
  char cwd[1024 * 1024] = {0};
  if (getcwd(cwd, sizeof(cwd)) != nullptr) {
    std::cout << "Current working dir: " << cwd << std::endl;
  } else {
    std::cerr << "Error calling getcwd()" << std::endl;
  }
}

}  // namespace util