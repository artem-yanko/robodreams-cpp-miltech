#include "driver/gptimer.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

constexpr i2c_port_num_t I2C_PORT = I2C_NUM_0;
constexpr gpio_num_t I2C_SDA = GPIO_NUM_8;
constexpr gpio_num_t I2C_SCL = GPIO_NUM_9;
constexpr uint8_t MPU6050_ADDRESS = 0x68;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t EXPECTED_WHO_AM_I = 0x68;

struct Sample {
    float ax{};
    float ay{};
    float az{};
    float gx{};
    float gy{};
    float gz{};
    float temp{};
};

volatile bool measurementTick = false;
volatile bool telemetryEnabled = true;
volatile uint32_t periodMs = 1000;
char modeName[16] = "normal";
i2c_master_bus_handle_t i2cBus = nullptr;
i2c_master_dev_handle_t mpu6050 = nullptr;
gptimer_handle_t measurementTimer = nullptr;

bool onTimerAlarm(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*) {
    measurementTick = true;
    return false;
}

esp_err_t writeRegister(uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    return i2c_master_transmit(mpu6050, data, sizeof(data), pdMS_TO_TICKS(100));
}

esp_err_t readRegisters(uint8_t reg, uint8_t* data, size_t size) {
    return i2c_master_transmit_receive(mpu6050, &reg, 1, data, size, pdMS_TO_TICKS(100));
}

int16_t readBigEndianInt16(const uint8_t* data, size_t offset) {
    uint16_t value = (static_cast<uint16_t>(data[offset]) << 8)
        | static_cast<uint16_t>(data[offset + 1]);
    return static_cast<int16_t>(value);
}

esp_err_t initI2c() {
    i2c_master_bus_config_t busConfig{};
    busConfig.i2c_port = I2C_PORT;
    busConfig.sda_io_num = I2C_SDA;
    busConfig.scl_io_num = I2C_SCL;
    busConfig.clk_source = I2C_CLK_SRC_DEFAULT;
    busConfig.glitch_ignore_cnt = 7;
    busConfig.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(&busConfig, &i2cBus);
    if (err != ESP_OK) {
        return err;
    }

    i2c_device_config_t deviceConfig{};
    deviceConfig.device_address = MPU6050_ADDRESS;
    deviceConfig.scl_speed_hz = 100000;

    return i2c_master_bus_add_device(i2cBus, &deviceConfig, &mpu6050);
}

esp_err_t initMpu6050() {
    uint8_t whoAmI = 0;
    esp_err_t err = readRegisters(REG_WHO_AM_I, &whoAmI, 1);
    if (err != ESP_OK) {
        return err;
    }

    std::printf("who_am_i=0x%02x\r\n", whoAmI);
    if (whoAmI != EXPECTED_WHO_AM_I) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return writeRegister(REG_PWR_MGMT_1, 0x00);
}

esp_err_t readSample(Sample& sample) {
    uint8_t data[14]{};
    esp_err_t err = readRegisters(REG_ACCEL_XOUT_H, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    const int16_t ax = readBigEndianInt16(data, 0);
    const int16_t ay = readBigEndianInt16(data, 2);
    const int16_t az = readBigEndianInt16(data, 4);
    const int16_t temp = readBigEndianInt16(data, 6);
    const int16_t gx = readBigEndianInt16(data, 8);
    const int16_t gy = readBigEndianInt16(data, 10);
    const int16_t gz = readBigEndianInt16(data, 12);

    sample.ax = static_cast<float>(ax) / 16384.0f;
    sample.ay = static_cast<float>(ay) / 16384.0f;
    sample.az = static_cast<float>(az) / 16384.0f;
    sample.temp = static_cast<float>(temp) / 340.0f + 36.53f;
    sample.gx = static_cast<float>(gx) / 131.0f;
    sample.gy = static_cast<float>(gy) / 131.0f;
    sample.gz = static_cast<float>(gz) / 131.0f;
    return ESP_OK;
}

esp_err_t applyTimerPeriod(uint32_t period) {
    gptimer_alarm_config_t alarmConfig{};
    alarmConfig.alarm_count = static_cast<uint64_t>(period) * 1000;
    alarmConfig.reload_count = 0;
    alarmConfig.flags.auto_reload_on_alarm = true;

    return gptimer_set_alarm_action(measurementTimer, &alarmConfig);
}

esp_err_t initTimer() {
    gptimer_config_t timerConfig{};
    timerConfig.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timerConfig.direction = GPTIMER_COUNT_UP;
    timerConfig.resolution_hz = 1000000;

    esp_err_t err = gptimer_new_timer(&timerConfig, &measurementTimer);
    if (err != ESP_OK) {
        return err;
    }

    gptimer_event_callbacks_t callbacks{};
    callbacks.on_alarm = onTimerAlarm;
    err = gptimer_register_event_callbacks(measurementTimer, &callbacks, nullptr);
    if (err != ESP_OK) {
        return err;
    }

    err = applyTimerPeriod(periodMs);
    if (err != ESP_OK) {
        return err;
    }

    err = gptimer_enable(measurementTimer);
    if (err != ESP_OK) {
        return err;
    }

    return gptimer_start(measurementTimer);
}

void printSample(const Sample& sample) {
    std::printf(
        "t=%lld ms ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f temp=%.1f mode=%s period=%lu\r\n",
        esp_timer_get_time() / 1000,
        sample.ax,
        sample.ay,
        sample.az,
        sample.gx,
        sample.gy,
        sample.gz,
        sample.temp,
        modeName,
        static_cast<unsigned long>(periodMs)
    );
}

void handleCommand(char* line) {
    int value = 0;
    char command[16]{};
    char text[16]{};

    if (std::sscanf(line, " %15s", command) != 1) {
        return;
    }

    if (std::strcmp(command, "p") == 0) {
        if (std::sscanf(line, " %*s %d", &value) != 1) {
            std::printf("err period_missing\r\n");
            return;
        }

        if (value < 100 || value > 5000) {
            std::printf("err period_range\r\n");
            return;
        }

        const uint32_t newPeriod = static_cast<uint32_t>(value);
        const esp_err_t err = applyTimerPeriod(newPeriod);
        if (err != ESP_OK) {
            std::printf("err timer_period=%s\r\n", esp_err_to_name(err));
            return;
        }

        periodMs = newPeriod;
        std::printf("ok period=%lu\r\n", static_cast<unsigned long>(periodMs));
        return;
    }

    if (std::strcmp(command, "mode") == 0) {
        if (std::sscanf(line, " %*s %15s", text) != 1) {
            std::printf("err mode_missing\r\n");
            return;
        }

        std::strncpy(modeName, text, sizeof(modeName) - 1);
        modeName[sizeof(modeName) - 1] = '\0';
        std::printf("ok mode=%s\r\n", modeName);
        return;
    }

    if (std::strcmp(command, "q") == 0) {
        telemetryEnabled = !telemetryEnabled;
        std::printf("ok telemetry=%s\r\n", telemetryEnabled ? "on" : "off");
        return;
    }

    std::printf("err unknown_command=%s\r\n", command);
}

void pollUartCommands() {
    static char line[64]{};
    static size_t size = 0;

    int ch = 0;
    while ((ch = std::getchar()) != EOF) {
        if (ch == '\r') {
            continue;
        }

        if (ch == '\n') {
            line[size] = '\0';
            handleCommand(line);
            size = 0;
            continue;
        }

        if (size + 1 < sizeof(line)) {
            line[size++] = static_cast<char>(ch);
        }
    }
}

} // namespace

extern "C" void app_main(void) {
    setvbuf(stdin, nullptr, _IONBF, 0);
    setvbuf(stdout, nullptr, _IONBF, 0);

    std::printf("homework_17 esp32s3 mpu6050 uart ready\r\n");

    esp_err_t err = initI2c();
    if (err != ESP_OK) {
        std::printf("err i2c_init=%s\r\n", esp_err_to_name(err));
        return;
    }

    err = initMpu6050();
    if (err != ESP_OK) {
        std::printf("err mpu6050_init=%s\r\n", esp_err_to_name(err));
        return;
    }

    err = initTimer();
    if (err != ESP_OK) {
        std::printf("err timer_init=%s\r\n", esp_err_to_name(err));
        return;
    }

    while (true) {
        pollUartCommands();

        if (measurementTick) {
            measurementTick = false;

            Sample sample{};
            err = readSample(sample);
            if (err == ESP_OK && telemetryEnabled) {
                printSample(sample);
            } else if (err != ESP_OK) {
                std::printf("err read_sample=%s\r\n", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
