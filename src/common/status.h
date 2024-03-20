#pragma once

#include <iostream>

#define USE_LOG true

#define DEBUG_LOG                                                              \
  if(USE_LOG)                                                                  \
  {                                                                            \
    std::cout << __FILE__ << "  " << __FUNCTION__ << "   " << __LINE__         \
              << "\n";                                                         \
  }
#define LOG(param) std::cout << param << "\n";

namespace beacon
{
enum class Status
{
  OK,
  ERROR
};
} // namespace beacon