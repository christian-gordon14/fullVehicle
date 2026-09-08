#include "imu.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "nvs.h"

// ============================================================
// Configuration
// ============================================================

#define SCL_PIN                 22
#define SDA_PIN                 21
#define LSM6DOX_ADDR_PRIMARY    0x6A
#define LSM6DOX_ADDR_SECONDARY  0x6B

#define WHO_AM_I                0x0F
#define CTRL1_XL                0x10
#define CTRL2_G                 0x11

#define OUTX_L_G                0x22
#define OUTX_L_A                0x28

#define LPF_ALPHA               0.1f
#define CALIBRATION_SAMPLES     10000

#define AX_SCALE 0.000061f
#define GY_SCALE 0.00875f

#define DT 0.001f
#define PI 3.14159f

// ============================================================
// Private variables
// ============================================================

static const char *TAG = "IMU";

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t imu_handle;
static bool imu_ready = false;

static AccelValues accel_filtered = {0};
static GyroValues gyro_filtered = {0};
static VehicleStates vehicleStates = {0};

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
static bool recalibrate_IMU = false;
static bool calibration_saved = false;
static bool calibration_complete = false;

typedef struct
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} ImuCalibration;

// ============================================================
// Private variables
// ============================================================
float yaw_heading = 0.f;

// ============================================================
// Function definitions
// ============================================================

static void init_i2c(void);
static esp_err_t add_imu_device(uint16_t address);
static void write_register(uint8_t reg, uint8_t value);
static esp_err_t read_register(uint8_t reg, uint8_t *data, size_t len);
static void low_pass_filter(float current_sample, float *filtered_value);
static void calibrate(float ax, float ay, float az, float gx, float gy, float gz);
static bool load_calibration(void);
static bool save_calibration(void);
static void scan_i2c(void);

// ============================================================
// Private functions
// ============================================================

static void scan_i2c(void)
{
    for (uint8_t address = 1; address <= 0x77; address++)
    {
        esp_err_t result =
            i2c_master_probe(bus_handle, address, 100);

        if (result == ESP_OK)
        {
            ESP_LOGI(TAG, "I2C device found at 0x%02X", address);
        }
    }
}

static bool load_calibration(void)
{
    ImuCalibration calibration;
    size_t calibration_size = sizeof(calibration);
    nvs_handle_t handle;

    esp_err_t result = nvs_open("imu", NVS_READONLY, &handle);

    if (result != ESP_OK)
    {
        return false;
    }

    result = nvs_get_blob(
        handle,
        "calibration",
        &calibration,
        &calibration_size
    );
    nvs_close(handle);

    if (result != ESP_OK || calibration_size != sizeof(calibration))
    {
        return false;
    }

    ax_bias = calibration.ax;
    ay_bias = calibration.ay;
    az_bias = calibration.az;
    gx_bias = calibration.gx;
    gy_bias = calibration.gy;
    gz_bias = calibration.gz;

    return true;
}

static bool save_calibration(void)
{
    const ImuCalibration calibration = {
        ax_bias, ay_bias, az_bias,
        gx_bias, gy_bias, gz_bias
    };
    nvs_handle_t handle;

    esp_err_t result = nvs_open("imu", NVS_READWRITE, &handle);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(result));
        return false;
    }

    result = nvs_set_blob(
        handle,
        "calibration",
        &calibration,
        sizeof(calibration)
    );

    if (result == ESP_OK)
    {
        result = nvs_commit(handle);
    }

    nvs_close(handle);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Calibration save failed: %s",
            esp_err_to_name(result)
        );
        return false;
    }

    return true;
}

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
    scan_i2c();
    esp_err_t result = add_imu_device(LSM6DOX_ADDR_PRIMARY);

    if (result != ESP_OK)
    {
        ESP_LOGW(TAG, "No IMU at 0x%02X, trying 0x%02X",
                 LSM6DOX_ADDR_PRIMARY, LSM6DOX_ADDR_SECONDARY);
        ESP_ERROR_CHECK(add_imu_device(LSM6DOX_ADDR_SECONDARY));
    }
}

static esp_err_t add_imu_device(uint16_t address)
{
    i2c_device_config_t imu_config =
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };

    return i2c_master_bus_add_device(
        bus_handle,
        &imu_config,
        &imu_handle
    );
}

static void write_register(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    ESP_ERROR_CHECK(i2c_master_transmit(imu_handle, data, 2, -1));
}

static esp_err_t read_register(uint8_t reg, uint8_t *data, size_t len)
{
    if (data == NULL || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(
        imu_handle,
        &reg,
        1,
        data,
        len,
        -1
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

    if (sample_count == CALIBRATION_SAMPLES)
    {
        ax_bias = ax_total / CALIBRATION_SAMPLES;
        ay_bias = ay_total / CALIBRATION_SAMPLES;
        az_bias = (az_total / CALIBRATION_SAMPLES) - 1.0f;

        gx_bias = gx_total / CALIBRATION_SAMPLES;
        gy_bias = gy_total / CALIBRATION_SAMPLES;
        gz_bias = gz_total / CALIBRATION_SAMPLES;

        if (!calibration_saved)
        {
            calibration_saved = save_calibration();

            if (calibration_saved)
            {
                ESP_LOGI(TAG, "IMU calibration saved to NVS");
            }
        }

        calibration_complete = true;
        ESP_LOGI(
            TAG,
            "IMU calibration complete: accel bias=(%.4f, %.4f, %.4f), "
            "gyro bias=(%.4f, %.4f, %.4f)",
            ax_bias,
            ay_bias,
            az_bias,
            gx_bias,
            gy_bias,
            gz_bias
        );
    }
}

// ============================================================
// Public functions
// ============================================================

void imu_init(void)
{
    imu_ready = false;
    init_i2c();

    uint8_t who_am_i = 0;

    esp_err_t result = read_register(WHO_AM_I, &who_am_i, 1);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s", esp_err_to_name(result));
        return;
    }

    if (who_am_i == 0x6C)
    {
        ESP_LOGI(TAG, "LSM6DOX detected");
    }
    else
    {
        ESP_LOGE(TAG, "WHO_AM_I wrong: 0x%02X", who_am_i);
        return;
    }

    write_register(CTRL1_XL, 0b10000010);
    write_register(CTRL2_G, 0b10000000);

    if (!recalibrate_IMU && load_calibration())
    {
        calibration_saved = true;
        calibration_complete = true;
        ESP_LOGI(TAG, "Loaded IMU calibration from NVS");
    }
    else
    {
        sample_count = 0;
        calibration_saved = false;
        calibration_complete = false;
        ax_total = 0.0f;
        ay_total = 0.0f;
        az_total = 0.0f;
        gx_total = 0.0f;
        gy_total = 0.0f;
        gz_total = 0.0f;

        ESP_LOGI(TAG, "Starting IMU calibration");
    }

    imu_ready = true;
}

void imu_update(void)
{
    if (!imu_ready)
    {
        return;
    }

    uint8_t accel_data[6];
    uint8_t gyro_data[6];

    if (read_register(OUTX_L_A, accel_data, sizeof(accel_data)) != ESP_OK)
    {
        ESP_LOGW(TAG, "Accelerometer read failed");
        return;
    }

    if (read_register(OUTX_L_G, gyro_data, sizeof(gyro_data)) != ESP_OK)
    {
        ESP_LOGW(TAG, "Gyroscope read failed");
        return;
    }   

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

    if (!calibration_complete)
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

    low_pass_filter(gx, &gyro_filtered.gx);
    low_pass_filter(gy, &gyro_filtered.gy);
    low_pass_filter(gz, &gyro_filtered.gz);

    vehicleStates.pitch = vehicleStates.pitch + DT * gyro_filtered.gx;
    float ax_gravity = sinf(vehicleStates.pitch * PI / 180.0f);

    low_pass_filter(ax - ax_gravity, &accel_filtered.ax);
    low_pass_filter(ay, &accel_filtered.ay);
    low_pass_filter(az, &accel_filtered.az);


    // printf("%.2f, %.2f, %.2f\n", ax, ay, az);

    // ACCEL CONVERSION TO VELOCITY
    vehicleStates.xVelocity += DT * accel_filtered.ax * 9.81f;
    vehicleStates.yVelocity += DT * accel_filtered.ay * 9.81f;
    // VELOCITY CONVERSION TO POSITION
    vehicleStates.xPosition += DT * (vehicleStates.xVelocity * cosf(vehicleStates.heading * PI / (180.f)) - vehicleStates.yVelocity * sinf(vehicleStates.heading * PI / (180.f)));
    vehicleStates.yPosition += DT * (vehicleStates.xVelocity * sinf(vehicleStates.heading * PI / (180.f)) + vehicleStates.yVelocity * cosf(vehicleStates.heading * PI / (180.f)));
    // GYRO TO HEADING
    vehicleStates.heading += DT * gyro_filtered.gz;
}

AccelValues imu_get_accel(void)
{
    return accel_filtered;
}

GyroValues imu_get_gyro(void)
{
    return gyro_filtered;
}

VehicleStates imu_get_states(void)
{
    return vehicleStates;
}