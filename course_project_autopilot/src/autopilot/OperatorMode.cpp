#include "autopilot/OperatorMode.hpp"

const char* operatorModeName(OperatorMode mode) {
    switch (mode) {
    case OperatorMode::Manual:
        return "MANUAL";
    case OperatorMode::Auto:
        return "AUTO";
    }

    return "UNKNOWN";
}
