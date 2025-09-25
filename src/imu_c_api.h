#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ImuHandle ImuHandle;

typedef struct {
        double timestamp;
        float  accel_x, accel_y, accel_z;
        float  gyro_x, gyro_y, gyro_z;
        float  mag_x, mag_y, mag_z;
        float  yaw, pitch, roll;
} ImuData;

__attribute__((visibility("default"))) ImuHandle *imu_init(void);

__attribute__((visibility("default"))) void       imu_deinit(ImuHandle *imu);

__attribute__((visibility("default"))) int        imu_read(ImuHandle *imu, ImuData *out);

#ifdef __cplusplus
}
#endif
