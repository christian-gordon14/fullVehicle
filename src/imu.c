#include "imu.h"
#include <stdint.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#define SCL_PIN                 22
#define SDA_PIN                 21
#define LSM6DOX_ADDR            0x6A

#define WHO_AM_I                0x0F
#define CTRL1_XL                0x10
#define CTRL2_G                 0x11

#define OUTX_L_G                0x22
#define OUTX_L_A                0x28

#define LPF_ALPHA               0.1f
#define CALIBRATION_SAMPLES     1000

#define AX_SCALE 0.000061f;
#define GY_SCALE 0.00875f;

static const char *TAG = "IMU";

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t imu_handle;

static AccelValues accel_filtered = {0};
static GyroValues gyro_filtered = {0};

static float ax_bias = 0.0f;
static float ay_bias = 0.0f;
static float az_bias = 0.0f;
static float gx_bias = 0.0f;
static float gy_bias = 0.0f;
static float gz_bias = 0.0f;

static float ax_total = 0.0f;
static float ay_total = 0.0f;
static float az_total = 0.0f;
static float gx_total = 0.0f;
static float gy_total = 0.0f;
static float gz_total = 0.0f;

static int sample_count = 0;

static void init_i2c(void);
static void write_register(uint8_t reg, uint8_t value);
static void read_register(uint8_t reg, uint8_t *data, uint8_t len);
static void low_pass_filter(float current_sample, float *filtered_value);
static void calibrate(float ax, float ay, float az, float gx, float gy, float gz);

static void init_i2c(void)
{
    i2c_master_bus_config_t bus_config =
    {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = SCL_PIN,
        .sda_io_num = SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t imu_config =
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LSM6DOX_ADDR,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &imu_config, &imu_handle));
}

static void write_register(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    ESP_ERROR_CHECK(i2c_master_transmit(imu_handle, data, 2, -1));
}

static void read_register(uint8_t reg, uint8_t *data, uint8_t len)
{
    ESP_ERROR_CHECK(
        i2c_master_transmit_receive(
            imu_handle,
            &reg,
            1,
            data,
            len,
            -1
        )
    );
}

static void low_pass_filter(float current_sample, float *filtered_value)
{
    *filtered_value = LPF_ALPHA * current_sample +
                      (1.0f - LPF_ALPHA) * (*filtered_value);
}

static void calibrate(float ax, float ay, float az, float gx, float gy, float gz)
{
    sample_count++;

    ax_total += ax;
    ay_total += ay;
    az_total += az;
    gx_total += gx;
    gy_total += gy;
    gz_total += gz;

    ax_bias = ax_total / sample_count;
    ay_bias = ay_total / sample_count;
    az_bias = (az_total / sample_count) - 1.0f;

    gx_bias = gx_total / sample_count;
    gy_bias = gy_total / sample_count;
    gz_bias = gz_total / sample_count;

    if (sample_count >= CALIBRATION_SAMPLES)
    {
        accel_filtered.ax = ax - ax_bias;
        accel_filtered.ay = ay - ay_bias;
        accel_filtered.az = az - az_bias;

        gyro_filtered.gx = gx - gx_bias;
        gyro_filtered.gy = gy - gy_bias;
        gyro_filtered.gz = gz - gz_bias;
    }
}

void imu_init(void)
{
    init_i2c();

    uint8_t who_am_i;
    read_register(WHO_AM_I, &who_am_i, 1);

    if (who_am_i == 0x6C)
    {
        ESP_LOGI(TAG, "LSM6DOX detected");
    }
    else
    {
        ESP_LOGE(TAG, "WHO_AM_I wrong: 0x%02X", who_am_i);
        return;
    }

    write_register(CTRL1_XL, 0b01000010);
    write_register(CTRL2_G, 0b01000000);
}

void imu_update(void)
{
    uint8_t accel_data[6];
    uint8_t gyro_data[6];

    read_register(OUTX_L_A, accel_data, 6);
    read_register(OUTX_L_G, gyro_data, 6);

    int16_t ax_raw = (int16_t)((accel_data[1] << 8) | accel_data[0]);
    int16_t ay_raw = (int16_t)((accel_data[3] << 8) | accel_data[2]);
    int16_t az_raw = (int16_t)((accel_data[5] << 8) | accel_data[4]);

    int16_t gx_raw = (int16_t)((gyro_data[1] << 8) | gyro_data[0]);
    int16_t gy_raw = (int16_t)((gyro_data[3] << 8) | gyro_data[2]);
    int16_t gz_raw = (int16_t)((gyro_data[5] << 8) | gyro_data[4]);

    float ax = ax_raw * AX_SCALE;
    float ay = ay_raw * AX_SCALE;
    float az = az_raw * AX_SCALE;

    float gx = gx_raw * GY_SCALE;
    float gy = gy_raw * GY_SCALE;
    float gz = gz_raw * GY_SCALE;

    if (sample_count < CALIBRATION_SAMPLES)
    {
        calibrate(ax, ay, az, gx, gy, gz);
        return;
    }

    ax -= ax_bias;
    ay -= ay_bias;
    az -= az_bias;

    gx -= gx_bias;
    gy -= gy_bias;
    gz -= gz_bias;

    low_pass_filter(ax, &accel_filtered.ax);
    low_pass_filter(ay, &accel_filtered.ay);
    low_pass_filter(az, &accel_filtered.az);

    low_pass_filter(gx, &gyro_filtered.gx);
    low_pass_filter(gy, &gyro_filtered.gy);
    low_pass_filter(gz, &gyro_filtered.gz);
    // printf("%.2f, %.2f, %.2f\n", ax, ay, az);
}

AccelValues imu_get_accel(void)
{
    return accel_filtered;
}

GyroValues imu_get_gyro(void)
{
    return gyro_filtered;
}