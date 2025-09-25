#include "imu_c_api.h"
#include "IMU_ICM20948.h"
#include <new>

// internal definition
struct ImuHandle {
        IMU impl;
};

ImuHandle *imu_init(void) {
        try {
                return new ImuHandle();
        } catch (...) {
                return nullptr;
        }
}

void imu_deinit(ImuHandle *imu) {
        delete imu;
}

int imu_read(ImuHandle *imu, ImuData *out) {
        if (!imu || !out) return -1;
        if (!imu->impl.readSensor()) return -2;

        out->accel_x   = imu->impl.get_accel_xout_f();
        out->accel_y   = imu->impl.get_accel_yout_f();
        out->accel_z   = imu->impl.get_accel_zout_f();

        out->gyro_x    = imu->impl.get_gyro_xout_f();
        out->gyro_y    = imu->impl.get_gyro_yout_f();
        out->gyro_z    = imu->impl.get_gyro_zout_f();

        out->mag_x     = imu->impl.get_mag_xout_f();
        out->mag_y     = imu->impl.get_mag_yout_f();
        out->mag_z     = imu->impl.get_mag_zout_f();

        out->yaw       = imu->impl.get_AHRS_yaw_f();
        out->pitch     = imu->impl.get_AHRS_pitch_f();
        out->roll      = imu->impl.get_AHRS_roll_f();
        out->timestamp = imu->impl.get_timestamp();

        return 0;
}
