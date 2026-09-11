#include "imu.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "nvs.h"
#include "esp_timer.h"

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

#define LPF_ALPHA_XL               0.3f
#define LPF_ALPHA_GYRO             0.3f
#define POSE_CALIBRATION_SAMPLES  3000
#define GYRO_CALIBRATION_SAMPLES   3000
#define GYRO_WINDOW_NORM 0.03f

#define AX_SCALE 0.000061f
#define GY_SCALE 0.00875f

#define PI 3.14159f

// calculated from the 6 positions
#define AX_BIAS (-0.00070508f)
#define AY_BIAS -0.03379302f
#define AZ_BIAS 0.01673724f

// #define GX_BIAS 0.03190474f
// #define GY_BIAS -0.46292060f
// #define GZ_BIAS -0.43648447f

// ============================================================
// Private variables
// ============================================================

static const char *TAG = "IMU";

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t imu_handle;
static bool imu_ready = false;

static AccelValues accel_filtered = {0};
static AccelValues accel_gravity_compensated = {0};
static GyroValues gyro_filtered = {0};
static VehicleStates vehicleStates = {0};

static bool pose_initialized;
static int pose_sample_count;
static float ax_total;
static float ay_total;
static float az_total;

static bool gyro_initialized;
static int gyro_sample_count;
static float gx_total;
static float gy_total;
static float gz_total;
static float gx_bias;
static float gy_bias;
static float gz_bias;

static float gyro_transient_window_four = 0.f;
static float gyro_transient_window_three = 0.f;
static float gyro_transient_window_two = 0.f;
static float gyro_transient_window_one = 0.f;
static float gyro_total_window = 0;

static float ax_gravity = 0.f;

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

// ============================================================
// Function definitions
// ============================================================

static void init_i2c(void);
static esp_err_t add_imu_device(uint16_t address);
static void write_register(uint8_t reg, uint8_t value);
static esp_err_t read_register(uint8_t reg, uint8_t *data, size_t len);
static void low_pass_filter(float current_sample, float *filtered_value, float LPF_ALPHA);
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
        .scl_speed_hz = 400000,
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

static void low_pass_filter(float current_sample, float *filtered_value, float LPF_ALPHA)
{
    *filtered_value = LPF_ALPHA * current_sample +
                      (1.0f - LPF_ALPHA) * (*filtered_value);
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

    imu_ready = true;

    pose_initialized = false;
    pose_sample_count = 0;
    ax_total = 0;
    ay_total = 0;
    az_total = 0;
    vehicleStates.pitch = 0.0f;
    vehicleStates.roll = 0.0f;

    gyro_initialized = false;
    gyro_sample_count = 0;
    gx_total = 0;
    gy_total = 0;
    gz_total = 0;
}

void imu_update(void)
{
    if (!imu_ready)
    {
        return;
    }

    // TIMING
    static int64_t last_time = 0;
    int64_t start_time = esp_timer_get_time();
    float dt = 0.001f;

    if (last_time != 0)
    {
        dt = (start_time - last_time) * 1e-6f; 
    }
    
    last_time = start_time;

    uint8_t sensor_data[12];

    if (read_register(OUTX_L_G, sensor_data, sizeof(sensor_data)) != ESP_OK)
    {
        ESP_LOGW(TAG, "IMU read failed");
        return;
    }

    int16_t gx_raw = (int16_t)((sensor_data[1] << 8) | sensor_data[0]);
    int16_t gy_raw = (int16_t)((sensor_data[3] << 8) | sensor_data[2]);
    int16_t gz_raw = (int16_t)((sensor_data[5] << 8) | sensor_data[4]);

    int16_t ax_raw = (int16_t)((sensor_data[7] << 8) | sensor_data[6]);
    int16_t ay_raw = (int16_t)((sensor_data[9] << 8) | sensor_data[8]);
    int16_t az_raw = (int16_t)((sensor_data[11] << 8) | sensor_data[10]);

    float ax = ax_raw * AX_SCALE;
    float ay = ay_raw * AX_SCALE;
    float az = az_raw * AX_SCALE;

    float gx = gx_raw * GY_SCALE;
    float gy = gy_raw * GY_SCALE;
    float gz = gz_raw * GY_SCALE;

    ax -= AX_BIAS;
    ay -= AY_BIAS;
    az -= AZ_BIAS;

    if (!gyro_initialized)
    {
        gx_total += gx;
        gy_total += gy;
        gz_total += gz;
        gyro_sample_count++;

        if(gyro_sample_count >= GYRO_CALIBRATION_SAMPLES)
        {
            gx_bias = gx_total / GYRO_CALIBRATION_SAMPLES;
            gy_bias = gy_total / GYRO_CALIBRATION_SAMPLES;
            gz_bias = gz_total / GYRO_CALIBRATION_SAMPLES;
            gyro_initialized = true;
        }
        return;
    }

    gx -= gx_bias;
    gy -= gy_bias;
    gz -= gz_bias;

    if (!pose_initialized)
    {
        ax_total += ax;
        ay_total += ay;
        az_total += az;
        pose_sample_count++;

        if (pose_sample_count >= POSE_CALIBRATION_SAMPLES)
        {
            float ax_avg = ax_total / POSE_CALIBRATION_SAMPLES;
            float ay_avg = ay_total / POSE_CALIBRATION_SAMPLES;
            float az_avg = az_total / POSE_CALIBRATION_SAMPLES;
            vehicleStates.pitch = atan2f(ax_avg, az_avg) * 180.f / PI;
            vehicleStates.roll = atan2f(ay_avg, az_avg) * 180.f / PI;
            pose_initialized = true;
        }

        return;
    }

    low_pass_filter(gx, &gyro_filtered.gx, LPF_ALPHA_GYRO);
    low_pass_filter(gy, &gyro_filtered.gy, LPF_ALPHA_GYRO);
    low_pass_filter(gz, &gyro_filtered.gz, LPF_ALPHA_GYRO);

    low_pass_filter(ax, &accel_filtered.ax, LPF_ALPHA_XL);
    low_pass_filter(ay, &accel_filtered.ay, LPF_ALPHA_XL);
    low_pass_filter(az, &accel_filtered.az, LPF_ALPHA_XL);

    float pitch_accel = atan2f(accel_filtered.ax, sqrtf(accel_filtered.ay * accel_filtered.ay + accel_filtered.az * accel_filtered.az)) * 180.f / PI;
    float roll_accel = atan2f(accel_filtered.ay, sqrtf(accel_filtered.ax * accel_filtered.ax + accel_filtered.az * accel_filtered.az)) * 180.f / PI;

    float gyro_pitch = vehicleStates.pitch + dt * gyro_filtered.gy;
    float gyro_roll = vehicleStates.roll + dt * gyro_filtered.gx;


    // if gyro_pitch
    gyro_transient_window_one = gyro_transient_window_two;
    gyro_transient_window_two = gyro_transient_window_three;
    gyro_transient_window_three = gyro_transient_window_four;
    gyro_transient_window_four = gyro_filtered.gy;

    // sum of squares
    gyro_total_window = gyro_transient_window_four * gyro_transient_window_four + gyro_transient_window_three * gyro_transient_window_three + 
                    gyro_transient_window_two * gyro_transient_window_two + gyro_transient_window_one * gyro_transient_window_one;

    if (gyro_total_window > GYRO_WINDOW_NORM)
    {
        vehicleStates.pitch = gyro_pitch;
    }
    else
    {
        vehicleStates.pitch = 0.98f * gyro_pitch + 0.02 * pitch_accel;  
    }
    vehicleStates.roll = gyro_roll;
    
    ax_gravity = sinf(vehicleStates.pitch * PI / 180.f);
    float ay_gravity = sinf(vehicleStates.roll * PI / 180.f) * cosf(vehicleStates.pitch * PI / 180.f);
    float az_gravity = cosf(vehicleStates.roll * PI / 180.f) * cosf(vehicleStates.pitch * PI / 180.f);

    accel_gravity_compensated.ax = accel_filtered.ax - ax_gravity;
    accel_gravity_compensated.ay = accel_filtered.ay - ay_gravity;
    accel_gravity_compensated.az = accel_filtered.az;

    // ACCEL CONVERSION TO VELOCITY
    vehicleStates.xVelocity += dt * accel_gravity_compensated.ax * 9.81f;
    if (vehicleStates.xVelocity <= 0)
    {
        vehicleStates.xVelocity = 0;
    }

    vehicleStates.yVelocity += dt * accel_gravity_compensated.ay * 9.81f;
    // VELOCITY CONVERSION TO POSITION
    vehicleStates.xPosition += dt * (vehicleStates.xVelocity * cosf(vehicleStates.heading * PI / 180.f)
                                   - vehicleStates.yVelocity * sinf(vehicleStates.heading * PI / 180.f));

    vehicleStates.yPosition += dt * (vehicleStates.xVelocity * sinf(vehicleStates.heading * PI / 180.f)
                                   + vehicleStates.yVelocity * cosf(vehicleStates.heading * PI / 180.f));

    // GYRO TO HEADING
    vehicleStates.heading += dt * gyro_filtered.gz;
}

AccelValues imu_get_accel(void)
{
    return accel_gravity_compensated;
}

GyroValues imu_get_gyro(void)
{
    return gyro_filtered;
}

VehicleStates imu_get_states(void)
{
    return vehicleStates;
}

float imu_get_accel_terms(void)
{
    return ax_gravity;
}