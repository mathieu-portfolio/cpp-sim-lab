#pragma once
#include <cstdlib>
#include <iostream>

#define SIM_ASSERT(cond) \
do { \
    if (!(cond)) { \
        std::cerr << "Assertion failed: " #cond \
                  << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
        std::abort(); \
    } \
} while(0)
