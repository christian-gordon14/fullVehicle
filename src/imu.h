#ifndef IMU_H
#define IMU_H

typedef struct {
    float ax;
    float ay;
    float az;
} AccelValues;

typedef struct {
    float gx;
    float gy;
    float gz;
} GyroValues;

void imu_init(void);
void imu_update(void);

AccelValues imu_get_accel(void);
GyroValues imu_get_gyro(void);

#endif