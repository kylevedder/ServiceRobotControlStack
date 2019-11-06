#pragma once
// Copyright 2019 kvedder@seas.upenn.edu
// School of Engineering and Applied Sciences,
// University of Pennsylvania
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================
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
