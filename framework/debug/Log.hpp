#pragma once
#include <iostream>

#define SIM_LOG_INFO(msg)  std::cout << "[INFO] " << msg << "\n"
#define SIM_LOG_WARN(msg)  std::cout << "[WARN] " << msg << "\n"
#define SIM_LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << "\n"
