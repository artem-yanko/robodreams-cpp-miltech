#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <cmath>

#define ENABLE_LOG 1
#define ENABLE_DEBUG 0
#define ENABLE_ERROR 1

#if ENABLE_LOG
  #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
  #define LOG(msg)
#endif

#if ENABLE_DEBUG
  #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG(msg)
#endif

#if ENABLE_ERROR
  #define ERROR_LOG(msg) std::cerr << "[ERROR] " << msg << std::endl
#else
  #define ERROR_LOG(msg)
#endif

const int ticks_per_revolution = 1024;
const double wheel_radius_m = 0.3;
const double wheelbase_m = 1.0;
const double distance_per_tick = 2.0 * M_PI * wheel_radius_m / ticks_per_revolution;

int main(int argc, char** argv) {
    if (argc != 2) {
        ERROR_LOG("Define filepath as argument.\n   Usage: ugv_odometry <input_path>");
        return 1;
    }

    const char* INPUT_FILENAME = argv[1];
    std::fstream inputFile(INPUT_FILENAME);
    if (!inputFile){
        ERROR_LOG("Filename: " << INPUT_FILENAME << " not found");
        return 1;
    }

    long timestamp_ms = 0;
    long fl_ticks = 0, fr_ticks = 0, bl_ticks = 0, br_ticks = 0;
    if (!(inputFile >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks)) {
        ERROR_LOG("Filename: " << INPUT_FILENAME << " has invalid data format");
        return 1; 
    }
    LOG("Reading data from the file: " << INPUT_FILENAME);

    long prev_fl_ticks = fl_ticks;
    long prev_fr_ticks = fr_ticks;
    long prev_bl_ticks = bl_ticks;
    long prev_br_ticks = br_ticks;
    double x = 0;
    double y = 0;
    double theta = 0;
    int line_count = 0;
    while (inputFile >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks) {
        line_count++;

        long d_fl = fl_ticks - prev_fl_ticks;
        long d_fr = fr_ticks - prev_fr_ticks;
        long d_bl = bl_ticks - prev_bl_ticks;
        long d_br = br_ticks - prev_br_ticks;

        double d_left = (d_fl + d_bl) / 2.0;
        double d_right = (d_fr + d_br) / 2.0;
        double dL = d_left * distance_per_tick;
        double dR = d_right * distance_per_tick;

        double d = (dL + dR) / 2.0;
        double d_theta = (dR - dL) / wheelbase_m;
        x += d * cos(theta + d_theta/2);
        y += d * sin(theta + d_theta/2);
        theta += d_theta;

        DEBUG("Data: " << timestamp_ms << " " << x << " " << y << " " << theta << "\n");
        // Exact output needed for HW4
        std::cout << timestamp_ms << " " << x << " " << y << " " << theta << std::endl;
        prev_fl_ticks = fl_ticks;
        prev_fr_ticks = fr_ticks;
        prev_bl_ticks = bl_ticks;
        prev_br_ticks = br_ticks;

    }
    LOG("Calculations finished! Total lines processed: " << line_count);


    return 0;
}
