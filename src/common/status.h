#pragma once

#include <iostream>

#define DEBUG_LOG                                                              \
  std::cout << __FILE__ << "  " << __FUNCTION__ << "   " << __LINE__ << "\n";
#define LOG(param) std::cout << param << "\n";

namespace beacon
{
enum class Status
{
  OK,
  ERROR
};
} // namespace beacon