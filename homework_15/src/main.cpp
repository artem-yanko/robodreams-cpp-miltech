#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr uint8_t MPU6050_DEFAULT_ADDRESS = 0x68;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t EXPECTED_WHO_AM_I = 0x68;

struct ProgramOptions {
    std::string bus{"/dev/i2c-1"};
    int address{MPU6050_DEFAULT_ADDRESS};
    int samples{20};
    int intervalMs{250};
};

struct Mpu6050Sample {
    double accelXg{};
    double accelYg{};
    double accelZg{};
    double temperatureC{};
    double gyroXdps{};
    double gyroYdps{};
    double gyroZdps{};
};

class FileDescriptor {
public:
    explicit FileDescriptor(int fd) : fd(fd) {}

    ~FileDescriptor() {
        if (fd >= 0) {
            close(fd);
        }
    }

    FileDescriptor(const FileDescriptor&) = delete;
    FileDescriptor& operator=(const FileDescriptor&) = delete;

    int get() const {
        return fd;
    }

private:
    int fd;
};

int parseInteger(const std::string& value) {
    size_t parsedChars = 0;
    int result = std::stoi(value, &parsedChars, 0);
    if (parsedChars != value.size()) {
        throw std::invalid_argument("invalid integer: " + value);
    }
    return result;
}

std::string toHex(int value) {
    std::ostringstream stream;
    stream << "0x" << std::hex << std::uppercase << value;
    return stream.str();
}

ProgramOptions parseArgs(int argc, char* argv[]) {
    ProgramOptions options{};

    if (argc > 1) {
        options.bus = argv[1];
    }

    if (argc > 2) {
        options.address = parseInteger(argv[2]);
    }

    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--samples" && i + 1 < argc) {
            options.samples = parseInteger(argv[++i]);
        } else if (arg == "--interval-ms" && i + 1 < argc) {
            options.intervalMs = parseInteger(argv[++i]);
        } else {
            throw std::invalid_argument("unknown or incomplete argument: " + arg);
        }
    }

    if (options.address < 0x03 || options.address > 0x77) {
        throw std::invalid_argument("I2C address must be in range 0x03..0x77");
    }
    if (options.samples <= 0) {
        throw std::invalid_argument("samples must be positive");
    }
    if (options.intervalMs <= 0) {
        throw std::invalid_argument("interval-ms must be positive");
    }

    return options;
}

FileDescriptor openI2cDevice(const ProgramOptions& options) {
    int fd = open(options.bus.c_str(), O_RDWR);
    if (fd < 0) {
        throw std::runtime_error("failed to open " + options.bus + ": " + std::strerror(errno));
    }

    if (ioctl(fd, I2C_SLAVE, options.address) < 0) {
        const std::string message = "failed to select I2C address "
            + toHex(options.address) + ": " + std::strerror(errno);
        close(fd);
        throw std::runtime_error(message);
    }

    return FileDescriptor(fd);
}

void writeRegister(int fd, uint8_t reg, uint8_t value) {
    const uint8_t buffer[2] = {reg, value};
    ssize_t written = write(fd, buffer, sizeof(buffer));
    if (written != static_cast<ssize_t>(sizeof(buffer))) {
        throw std::runtime_error("failed to write register " + toHex(reg)
            + ": " + std::strerror(errno));
    }
}

std::vector<uint8_t> readRegisters(int fd, uint8_t startReg, size_t count) {
    ssize_t written = write(fd, &startReg, 1);
    if (written != 1) {
        throw std::runtime_error("failed to set register pointer " + toHex(startReg)
            + ": " + std::strerror(errno));
    }

    std::vector<uint8_t> buffer(count);
    ssize_t readBytes = read(fd, buffer.data(), buffer.size());
    if (readBytes != static_cast<ssize_t>(buffer.size())) {
        throw std::runtime_error("failed to read " + std::to_string(count)
            + " bytes from register " + toHex(startReg)
            + ": " + std::strerror(errno));
    }

    return buffer;
}

uint8_t readRegister(int fd, uint8_t reg) {
    return readRegisters(fd, reg, 1).front();
}

int16_t readBigEndianInt16(const std::vector<uint8_t>& data, size_t offset) {
    uint16_t value = (static_cast<uint16_t>(data.at(offset)) << 8)
        | static_cast<uint16_t>(data.at(offset + 1));
    return static_cast<int16_t>(value);
}

Mpu6050Sample readSample(int fd) {
    const std::vector<uint8_t> raw = readRegisters(fd, REG_ACCEL_XOUT_H, 14);

    const int16_t accelX = readBigEndianInt16(raw, 0);
    const int16_t accelY = readBigEndianInt16(raw, 2);
    const int16_t accelZ = readBigEndianInt16(raw, 4);
    const int16_t temperature = readBigEndianInt16(raw, 6);
    const int16_t gyroX = readBigEndianInt16(raw, 8);
    const int16_t gyroY = readBigEndianInt16(raw, 10);
    const int16_t gyroZ = readBigEndianInt16(raw, 12);

    Mpu6050Sample sample{};
    sample.accelXg = static_cast<double>(accelX) / 16384.0;
    sample.accelYg = static_cast<double>(accelY) / 16384.0;
    sample.accelZg = static_cast<double>(accelZ) / 16384.0;
    sample.temperatureC = static_cast<double>(temperature) / 340.0 + 36.53;
    sample.gyroXdps = static_cast<double>(gyroX) / 131.0;
    sample.gyroYdps = static_cast<double>(gyroY) / 131.0;
    sample.gyroZdps = static_cast<double>(gyroZ) / 131.0;
    return sample;
}

void printSample(int index, const Mpu6050Sample& sample) {
    std::cout << std::fixed << std::setprecision(3)
              << "#" << std::setw(2) << index
              << " accel[g]=("
              << std::setw(7) << sample.accelXg << ", "
              << std::setw(7) << sample.accelYg << ", "
              << std::setw(7) << sample.accelZg << ")"
              << " gyro[deg/s]=("
              << std::setw(8) << sample.gyroXdps << ", "
              << std::setw(8) << sample.gyroYdps << ", "
              << std::setw(8) << sample.gyroZdps << ")"
              << " temp[C]=" << std::setw(7) << sample.temperatureC
              << '\n';
}

void printUsage(const char* programName) {
    std::cerr << "Usage: " << programName
              << " [/dev/i2c-N] [address] [--samples N] [--interval-ms N]\n"
              << "Example: " << programName << " /dev/i2c-1 0x68 --samples 20 --interval-ms 250\n";
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        ProgramOptions options = parseArgs(argc, argv);
        FileDescriptor i2c = openI2cDevice(options);

        uint8_t whoAmI = readRegister(i2c.get(), REG_WHO_AM_I);
        std::cout << "WHO_AM_I=0x" << std::hex << static_cast<int>(whoAmI) << std::dec << '\n';
        if (whoAmI != EXPECTED_WHO_AM_I) {
            std::cerr << "Unexpected MPU-6050 WHO_AM_I value. Expected 0x68.\n";
            return 2;
        }

        writeRegister(i2c.get(), REG_PWR_MGMT_1, 0x00);

        for (int i = 1; i <= options.samples; ++i) {
            printSample(i, readSample(i2c.get()));
            std::this_thread::sleep_for(std::chrono::milliseconds(options.intervalMs));
        }

        return 0;
    } catch (const std::exception& exception) {
        printUsage(argv[0]);
        std::cerr << "Error: " << exception.what() << '\n';
        return 1;
    }
}
