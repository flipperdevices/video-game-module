#include "cli_commands.h"
#include "../led.h"
// #include "../imu/imu_reg.hpp"
#include "../imu/ICM42688P_regs.h"
#include <vector>
#include <hardware/gpio.h>
#include "hardware/resets.h"
#include "hardware/clocks.h"
#include "hardware/spi.h"
#include <FreeRTOS.h>
#include <task.h>
#include <math.h>

#define ICM42688P_TIMEOUT 100

typedef struct {
    spi_inst_t* spi;
    const uint8_t gp_cs;
    const uint8_t gp_mosi;
    const uint8_t gp_miso;
    const uint8_t gp_sck;
} SPIBus;

static void delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static bool icm42688p_write_reg(SPIBus& spi, uint8_t addr, uint8_t value) {
    bool res = false;
    gpio_put(spi.gp_cs, 0);
    do {
        uint8_t cmd_data[2] = {(uint8_t)(addr & 0x7Fu), value};
        if(spi_write_blocking(spi.spi, cmd_data, 2) != 2) break;
        res = true;
    } while(0);
    gpio_put(spi.gp_cs, 1);
    return res;
}

static bool icm42688p_read_reg(SPIBus& spi, uint8_t addr, uint8_t* value) {
    bool res = false;
    gpio_put(spi.gp_cs, 0);
    do {
        uint8_t cmd_byte = addr | (1 << 7);
        if(spi_write_blocking(spi.spi, &cmd_byte, 1) != 1) break;
        if(spi_read_blocking(spi.spi, 0, value, 1) != 1) break;
        res = true;
    } while(0);
    gpio_put(spi.gp_cs, 1);
    return res;
}

static bool icm42688p_read_mem(SPIBus& spi, uint8_t addr, uint8_t* data, uint8_t len) {
    bool res = false;
    gpio_put(spi.gp_cs, 0);
    do {
        uint8_t cmd_byte = addr | (1 << 7);
        if(spi_write_blocking(spi.spi, &cmd_byte, 1) != 1) break;
        if(spi_read_blocking(spi.spi, 0, data, len) != len) break;
        res = true;
    } while(0);
    gpio_put(spi.gp_cs, 1);
    return res;
}

typedef enum {
    DataRate32kHz = 0x01,
    DataRate16kHz = 0x02,
    DataRate8kHz = 0x03,
    DataRate4kHz = 0x04,
    DataRate2kHz = 0x05,
    DataRate1kHz = 0x06,
    DataRate200Hz = 0x07,
    DataRate100Hz = 0x08,
    DataRate50Hz = 0x09,
    DataRate25Hz = 0x0A,
    DataRate12_5Hz = 0x0B,
    DataRate6_25Hz = 0x0C, // Accelerometer only
    DataRate3_125Hz = 0x0D, // Accelerometer only
    DataRate1_5625Hz = 0x0E, // Accelerometer only
    DataRate500Hz = 0x0F,
} ICM42688PDataRate;

typedef enum {
    AccelFullScale16G = 0,
    AccelFullScale8G,
    AccelFullScale4G,
    AccelFullScale2G,
    AccelFullScaleTotal,
} ICM42688PAccelFullScale;

typedef enum {
    GyroFullScale2000DPS = 0,
    GyroFullScale1000DPS,
    GyroFullScale500DPS,
    GyroFullScale250DPS,
    GyroFullScale125DPS,
    GyroFullScale62_5DPS,
    GyroFullScale31_25DPS,
    GyroFullScale15_625DPS,
    GyroFullScaleTotal,
} ICM42688PGyroFullScale;

static const struct AccelFullScale {
    float value;
    uint8_t reg_mask;
} accel_fs_modes[] = {
    [AccelFullScale16G] = {16.f, ICM42688_AFS_16G},
    [AccelFullScale8G] = {8.f, ICM42688_AFS_8G},
    [AccelFullScale4G] = {4.f, ICM42688_AFS_4G},
    [AccelFullScale2G] = {2.f, ICM42688_AFS_2G},
};

static const struct GyroFullScale {
    float value;
    uint8_t reg_mask;
} gyro_fs_modes[] = {
    [GyroFullScale2000DPS] = {2000.f, ICM42688_GFS_2000DPS},
    [GyroFullScale1000DPS] = {1000.f, ICM42688_GFS_1000DPS},
    [GyroFullScale500DPS] = {500.f, ICM42688_GFS_500DPS},
    [GyroFullScale250DPS] = {250.f, ICM42688_GFS_250DPS},
    [GyroFullScale125DPS] = {125.f, ICM42688_GFS_125DPS},
    [GyroFullScale62_5DPS] = {62.5f, ICM42688_GFS_62_5DPS},
    [GyroFullScale31_25DPS] = {31.25f, ICM42688_GFS_31_25DPS},
    [GyroFullScale15_625DPS] = {15.625f, ICM42688_GFS_15_625DPS},
};

struct ICM42688P {
    SPIBus bus;
    float accel_scale;
    float gyro_scale;
};

static bool icm42688p_accel_config(
    ICM42688P& imu,
    ICM42688PAccelFullScale full_scale,
    ICM42688PDataRate rate) {
    imu.accel_scale = accel_fs_modes[full_scale].value;
    uint8_t reg_value = accel_fs_modes[full_scale].reg_mask | rate;
    return icm42688p_write_reg(imu.bus, ICM42688_ACCEL_CONFIG0, reg_value);
}

static bool icm42688p_gyro_config(
    ICM42688P& imu,
    ICM42688PGyroFullScale full_scale,
    ICM42688PDataRate rate) {
    imu.gyro_scale = gyro_fs_modes[full_scale].value;
    uint8_t reg_value = gyro_fs_modes[full_scale].reg_mask | rate;
    return icm42688p_write_reg(imu.bus, ICM42688_GYRO_CONFIG0, reg_value);
}

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} __attribute__((packed)) ICM42688PRawData;

static bool icm42688p_read_accel_raw(ICM42688P& imu, ICM42688PRawData* data) {
    bool ret = icm42688p_read_mem(
        imu.bus, ICM42688_ACCEL_DATA_X1, (uint8_t*)data, sizeof(ICM42688PRawData));
    return ret;
}

static bool icm42688p_read_gyro_raw(ICM42688P& imu, ICM42688PRawData* data) {
    bool ret = icm42688p_read_mem(
        imu.bus, ICM42688_GYRO_DATA_X1, (uint8_t*)data, sizeof(ICM42688PRawData));
    return ret;
}

static void read_awg_gyro(ICM42688P& imu, int32_t* data) {
    int32_t sample_cnt = 0;
    size_t timeout = 300;

    do {
        delay_ms(1);
        uint8_t int_status = 0;
        icm42688p_read_reg(imu.bus, ICM42688_INT_STATUS, &int_status);

        if(int_status & (1 << 3)) {
            ICM42688PRawData raw_data;
            icm42688p_read_gyro_raw(imu, &raw_data);
            data[0] += (int32_t)raw_data.x;
            data[1] += (int32_t)raw_data.y;
            data[2] += (int32_t)raw_data.z;
            sample_cnt++;
        }
        timeout--;
    } while((sample_cnt < 200) && (timeout > 0));

    data[0] /= sample_cnt;
    data[1] /= sample_cnt;
    data[2] /= sample_cnt;
}

static void read_awg_accel(ICM42688P& imu, int32_t* data) {
    int32_t sample_cnt = 0;
    size_t timeout = 300;

    do {
        delay_ms(1);
        uint8_t int_status = 0;
        icm42688p_read_reg(imu.bus, ICM42688_INT_STATUS, &int_status);

        if(int_status & (1 << 3)) {
            ICM42688PRawData raw_data;
            icm42688p_read_accel_raw(imu, &raw_data);
            data[0] += (int32_t)raw_data.x;
            data[1] += (int32_t)raw_data.y;
            data[2] += (int32_t)raw_data.z;
            sample_cnt++;
        }
        timeout--;
    } while((sample_cnt < 200) && (timeout > 0));

    data[0] /= sample_cnt;
    data[1] /= sample_cnt;
    data[2] /= sample_cnt;
}

#define INV_ST_OTP_EQUATION(FS, ST_code) \
    (uint32_t)((2620.f / powf(2.f, 3.f - FS)) * powf(1.01f, ST_code - 1.f) + 0.5f)

/* Pass/Fail criteria */
#define MIN_RATIO 0.5f /* expected ratio greater than 0.5 */
#define MAX_RATIO 1.5f /* expected ratio lower than 1.5 */

#define MIN_ST_GYRO_DPS 60 /* expected values greater than 60dps */
#define MAX_ST_GYRO_OFFSET_DPS 20 /* expected offset less than 20 dps */

#define MIN_ST_ACCEL_MG 50 /* expected values in [50mgee;1200mgee] */
#define MAX_ST_ACCEL_MG 1200

static bool run_gyro_self_test(ICM42688P& imu, int32_t* st_bias, Cli* cli) {
    int32_t values_normal[3] = {0};
    int32_t values_selftest[3] = {0};
    uint32_t selftest_response[3];

    uint8_t selftest_data[3];
    uint32_t selftest_otp[3];

    bool result = true;

    icm42688p_gyro_config(imu, GyroFullScale250DPS, DataRate1kHz);
    icm42688p_write_reg(imu.bus, ICM42688_GYRO_CONFIG1, 0x1A);
    icm42688p_write_reg(imu.bus, ICM42688_GYRO_ACCEL_CONFIG0, 0x14);

    read_awg_gyro(imu, values_normal);

    icm42688p_write_reg(imu.bus, ICM42688_SELF_TEST_CONFIG, (1 << 0) | (1 << 1) | (1 << 2));
    delay_ms(200);

    read_awg_gyro(imu, values_selftest);

    icm42688p_write_reg(imu.bus, ICM42688_SELF_TEST_CONFIG, 0);

    for(uint8_t i = 0; i < 3; i++) {
        selftest_response[i] = abs(values_selftest[i] - values_normal[i]);
    }

    /* Read ST_DATA */
    icm42688p_write_reg(imu.bus, ICM42688_REG_BANK_SEL, 1);
    icm42688p_read_mem(imu.bus, ICM42688_XG_ST_DATA, selftest_data, 3);
    icm42688p_write_reg(imu.bus, ICM42688_REG_BANK_SEL, 0);

    /* If ST_DATA = 0 for any axis */
    if(selftest_data[0] == 0 || selftest_data[1] == 0 || selftest_data[2] == 0) {
        /* compare the Self-Test response to the ST absolute limits */
        for(uint8_t i = 0; i < 3; i++) {
            if(selftest_response[i] < (MIN_ST_GYRO_DPS * 32768 / 250)) {
                cli_printf(cli, "Gyro selftest error: limits" EOL);
                result = false;
            }
        }
    } else { /* If ST_DATA != 0 for all axis */
        /* compare the Self-Test response to the factory OTP values */
        for(uint8_t i = 0; i < 3; i++) {
            selftest_otp[i] = INV_ST_OTP_EQUATION(GyroFullScale250DPS, selftest_data[i]);
            if(selftest_otp[i] == 0) {
                cli_printf(cli, "Gyro selftest error: otp 0" EOL);
                result = false;
            } else {
                float ratio = ((float)selftest_response[i]) / ((float)selftest_otp[i]);
                if((ratio >= MAX_RATIO) || (ratio <= MIN_RATIO)) {
                    cli_printf(cli, "Gyro selftest error: otp ratio" EOL);
                    result = false;
                }
            }
        }
    }

    /* stored the computed bias (checking GST and GOFFSET values) */
    for(uint8_t i = 0; i < 3; i++) {
        if((abs(values_normal[i]) > (int32_t)(MAX_ST_GYRO_OFFSET_DPS * 32768 / 250))) {
            result = false;
            cli_printf(cli, "Gyro selftest error: bias" EOL);
        }

        st_bias[i] = values_normal[i];
    }

    return result;
}

static bool run_accel_self_test(ICM42688P& imu, int32_t* st_bias, Cli* cli) {
    int32_t values_normal[3] = {0};
    int32_t values_selftest[3] = {0};
    uint32_t selftest_response[3];

    uint8_t selftest_data[3];
    uint32_t selftest_otp[3];

    bool result = true;

    uint32_t accel_sensitivity_1g = 32768 / 2;

    icm42688p_accel_config(imu, AccelFullScale2G, DataRate1kHz);
    icm42688p_write_reg(imu.bus, ICM42688_ACCEL_CONFIG1, 0x15);
    icm42688p_write_reg(imu.bus, ICM42688_GYRO_ACCEL_CONFIG0, 0x41);

    read_awg_accel(imu, values_normal);

    icm42688p_write_reg(
        imu.bus, ICM42688_SELF_TEST_CONFIG, (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6));
    delay_ms(25);

    read_awg_accel(imu, values_selftest);

    icm42688p_write_reg(imu.bus, ICM42688_SELF_TEST_CONFIG, 0);

    for(uint8_t i = 0; i < 3; i++) {
        selftest_response[i] = abs(values_selftest[i] - values_normal[i]);
    }

    /* Read ST_DATA */
    icm42688p_write_reg(imu.bus, ICM42688_REG_BANK_SEL, 2);
    icm42688p_read_mem(imu.bus, ICM42688_XA_ST_DATA, selftest_data, 3);
    icm42688p_write_reg(imu.bus, ICM42688_REG_BANK_SEL, 0);

    /* If ST_DATA = 0 for any axis */
    if(selftest_data[0] == 0 || selftest_data[1] == 0 || selftest_data[2] == 0) {
        /* compare the Self-Test response to the ST absolute limits */
        for(uint8_t i = 0; i < 3; i++) {
            if((selftest_response[i] < ((MIN_ST_ACCEL_MG * accel_sensitivity_1g) / 1000)) ||
               (selftest_response[i] > ((MAX_ST_ACCEL_MG * accel_sensitivity_1g) / 1000))) {
                cli_printf(cli, "Accel selftest error: limits" EOL);
                result = false;
            }
        }
    } else { /* If ST_DATA != 0 for all axis */
        /* compare the Self-Test response to the factory OTP values */
        for(uint8_t i = 0; i < 3; i++) {
            selftest_otp[i] = INV_ST_OTP_EQUATION(AccelFullScale2G, selftest_data[i]);
            if(selftest_otp[i] == 0) {
                cli_printf(cli, "Accel selftest error: otp 0" EOL);
                result = false;
            } else {
                float ratio = ((float)selftest_response[i]) / ((float)selftest_otp[i]);
                if((ratio >= MAX_RATIO) || (ratio <= MIN_RATIO)) {
                    cli_printf(cli, "Accel selftest error: otp ratio" EOL);
                    result = false;
                }
            }
        }
    }

    /* stored the computed offset */
    for(uint8_t i = 0; i < 3; i++) {
        st_bias[i] = values_normal[i];
    }

    /* assume the largest data axis shows +1 or -1 G for gravity */
    uint8_t axis = 0;
    int8_t axis_sign = 1;
    if(abs(st_bias[1]) > abs(st_bias[0])) {
        axis = 1;
    }
    if(abs(st_bias[2]) > abs(st_bias[axis])) {
        axis = 2;
    }
    if(st_bias[axis] < 0) {
        axis_sign = -1;
    }

    st_bias[axis] -= accel_sensitivity_1g * axis_sign;

    return result;
}

static bool icm42688_selftest(ICM42688P& imu, bool* gyro_ok, bool* accel_ok, Cli* cli) {
    int32_t gyro_raw_bias[3] = {0};
    int32_t accel_raw_bias[3] = {0};

    *gyro_ok = run_gyro_self_test(imu, gyro_raw_bias, cli);
    if(*gyro_ok) {
        cli_printf(cli, "Gyro selftest: PASS" EOL);
    } else {
        cli_printf(cli, "Gyro selftest: FAIL" EOL);
    }

    *accel_ok = run_accel_self_test(imu, accel_raw_bias, cli);
    if(*accel_ok) {
        cli_printf(cli, "Accel selftest: PASS" EOL);
    } else {
        cli_printf(cli, "Accel selftest: FAIL" EOL);
    }

    cli_printf(
        cli,
        "Gyro bias: x=%ld, y=%ld, z=%ld" EOL,
        gyro_raw_bias[0],
        gyro_raw_bias[1],
        gyro_raw_bias[2]);

    cli_printf(
        cli,
        "Accel bias: x=%ld, y=%ld, z=%ld" EOL,
        accel_raw_bias[0],
        accel_raw_bias[1],
        accel_raw_bias[2]);

    return true;
}

static void spi_init(SPIBus& bus) {
    // Initialize CS pin high
    gpio_init(bus.gp_cs);
    gpio_set_dir(bus.gp_cs, GPIO_OUT);
    gpio_put(bus.gp_cs, 1);

    // Initialize SPI port at 1 MHz
    spi_init(bus.spi, 1000 * 1000);

    // Set SPI format
    spi_set_format(
        spi0, // SPI instance
        8, // Number of bits per transfer
        SPI_CPOL_1, // Polarity (CPOL)
        SPI_CPHA_1, // Phase (CPHA)
        SPI_MSB_FIRST);

    // Initialize SPI pins
    gpio_set_function(bus.gp_sck, GPIO_FUNC_SPI);
    gpio_set_function(bus.gp_mosi, GPIO_FUNC_SPI);
    gpio_set_function(bus.gp_miso, GPIO_FUNC_SPI);
}

static void spi_deinit(SPIBus& bus) {
    spi_deinit(spi0);
    gpio_set_function(bus.gp_sck, GPIO_FUNC_NULL);
    gpio_set_function(bus.gp_mosi, GPIO_FUNC_NULL);
    gpio_set_function(bus.gp_miso, GPIO_FUNC_NULL);
    gpio_set_function(bus.gp_cs, GPIO_FUNC_NULL);

    gpio_set_dir(bus.gp_sck, GPIO_IN);
    gpio_set_dir(bus.gp_mosi, GPIO_IN);
    gpio_set_dir(bus.gp_miso, GPIO_IN);
    gpio_set_dir(bus.gp_cs, GPIO_IN);
}

void cli_imu_test(Cli* cli, std::string& args) {
    // reg_read(spi, cs_pin, REG_DEVID, data, 1);

    const uint8_t cs_pin = 5;
    const uint8_t sck_pin = 2;
    const uint8_t mosi_pin = 3;
    const uint8_t miso_pin = 4;
    ICM42688P imu = {
        .bus =
            {
                .spi = spi0,
                .gp_cs = cs_pin,
                .gp_mosi = mosi_pin,
                .gp_miso = miso_pin,
                .gp_sck = sck_pin,
            },
        .accel_scale = 0,
        .gyro_scale = 0,
    };

    spi_init(imu.bus);

    do {
        // Software reset
        icm42688p_write_reg(imu.bus, ICM42688_REG_BANK_SEL, 0); // Set reg bank to 0
        icm42688p_write_reg(imu.bus, ICM42688_DEVICE_CONFIG, 0x01); // SPI Mode 0, SW reset
        delay_ms(10);

        uint8_t reg_value = 0;
        bool read_ok = icm42688p_read_reg(imu.bus, ICM42688_WHO_AM_I, &reg_value);
        if(!read_ok) {
            cli_printf(cli, "Chip ID: read failed" EOL);
            break;
        } else if(reg_value != ICM42688_WHOAMI) {
            cli_printf(
                cli, "Chip ID: wrong ID 0x%02X, expected 0x%02X" EOL, reg_value, ICM42688_WHOAMI);
            break;
        }

        cli_printf(cli, "Chip ID: 0x%02X" EOL, reg_value);

        // Disable all interrupts
        icm42688p_write_reg(imu.bus, ICM42688_INT_SOURCE0, 0);
        icm42688p_write_reg(imu.bus, ICM42688_INT_SOURCE1, 0);
        icm42688p_write_reg(imu.bus, ICM42688_INT_SOURCE3, 0);
        icm42688p_write_reg(imu.bus, ICM42688_INT_SOURCE4, 0);
        icm42688p_write_reg(imu.bus, ICM42688_REG_BANK_SEL, 4);
        icm42688p_write_reg(imu.bus, ICM42688_INT_SOURCE6, 0);
        icm42688p_write_reg(imu.bus, ICM42688_INT_SOURCE7, 0);
        icm42688p_write_reg(imu.bus, ICM42688_REG_BANK_SEL, 0);

        // Data format: little endian
        icm42688p_write_reg(imu.bus, ICM42688_INTF_CONFIG0, 0);

        // Enable all sensors
        icm42688p_write_reg(
            imu.bus,
            ICM42688_PWR_MGMT0,
            ICM42688_PWR_TEMP_ON | ICM42688_PWR_GYRO_MODE_LN | ICM42688_PWR_ACCEL_MODE_LN);
        delay_ms(50);

        icm42688p_accel_config(imu, AccelFullScale16G, DataRate1kHz);
        icm42688p_gyro_config(imu, GyroFullScale2000DPS, DataRate1kHz);

        bool gyro_ok, accel_ok;
        icm42688_selftest(imu, &gyro_ok, &accel_ok, cli);
        cli_printf(cli, "Gyro: %s" EOL, gyro_ok ? "OK" : "FAIL");
        cli_printf(cli, "Accel: %s" EOL, accel_ok ? "OK" : "FAIL");

    } while(false);

    spi_deinit(imu.bus);
}