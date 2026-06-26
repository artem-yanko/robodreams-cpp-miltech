#pragma once

#include <iostream>

#define LOG(message) std::cout << "[INFO] " << message << std::endl
#define ERROR_LOG(message) std::cerr << "[ERROR] " << message << std::endl
