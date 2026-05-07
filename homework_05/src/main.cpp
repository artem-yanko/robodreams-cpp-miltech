#include "telemetry.hpp"

#include <iostream>

int main(int argc, char** argv) {
    // The executable expects exactly one telemetry log path.
    if (argc != 2) {
        std::cerr << "usage: telemetry_check <input_path>\n";
        return 1;
    }

    Frame frames[MAX_TELEMETRY_FRAMES];
    const int frame_count = read_frames(argv[1], frames, MAX_TELEMETRY_FRAMES);
    // Exit when catch exception during validation.
    if (frame_count < 0) {
        return 1;
    }
    // Exit when telemetry log is empty.
    if (frame_count == 0) {
        std::cerr << "ERROR: empty telemetry log: " << argv[1] << '\n';
        return 1;
    }
    try {
        const Summary summary = summarize(frames, frame_count);
        print_summary(summary);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: failed to compute frame rate: " << "\n   "<< e.what() << '\n';
        return 1;
    }

    return 0;
}
