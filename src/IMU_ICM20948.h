#include <cstddef>
#include <stdint.h>

#define IMU_HAVE_AHRS  1
#define IMU_HAVE_9AXIS 1


#pragma pack(push, 1)
typedef struct IMUDATA {
        float  accel_xout  = 0;
        float  accel_yout  = 0;
        float  accel_zout  = 0;
        float  gyro_xout   = 0;
        float  gyro_yout   = 0;
        float  gyro_zout   = 0;
        float  AHRS_yaw    = 0;
        float  AHRS_pitch  = 0;
        float  AHRS_roll   = 0;
        float  AHRS_NQuatX = 0;
        float  AHRS_NQuatY = 0;
        float  AHRS_NQuatZ = 0;
        float  mag_xout    = 0;
        float  mag_yout    = 0;
        float  mag_zout    = 0;
        double timestamp   = 0;
} IMUDATA;
#pragma pack(pop)


//---------------------------------------------------------------------
// implement IMU class using icm20948 & TDK-Invensense sdk
//
class IMU {
      private:
        size_t tries;
        int    i2cFd    = 0;
        int    I2C_ADDR = 0x68;
        /* FSR configurations */
        int32_t cfg_acc_fsr = 16;  // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
        int32_t cfg_gyr_fsr = 500; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000
        /*mounting configuration*/
        const float cfg_mounting_matrix[9] = {1.f, 0, 0, 0, 1.f, 0, 0, 0, 1.f};
        int         unscaled_bias[3 * 2];

        void       *icm_device;
        IMUDATA     imudata;
        void        icm20948_apply_mounting_matrix(void);
        void        icm20948_set_fsr(void);
        int         icm20948_sensor_setup(void);
        void        setup();
        uint8_t     icm20948_get_grv_accuracy(void);
        void        ToEulerAngles(float *q, float *a);

      public:
        IMU();
        ~IMU();

        bool    readSensor();

        int16_t get_accel_xout() {
                return (imudata.accel_xout * 32767 / cfg_acc_fsr);
        };
        int16_t get_accel_yout() {
                return (imudata.accel_yout * 32767 / cfg_acc_fsr);
        };
        int16_t get_accel_zout() {
                return (imudata.accel_zout * 32767 / cfg_acc_fsr);
        };
        int16_t get_gyro_xout() {
                return (imudata.gyro_xout * 32767 / cfg_gyr_fsr);
        };
        int16_t get_gyro_yout() {
                return (imudata.gyro_yout * 32767 / cfg_gyr_fsr);
        };
        int16_t get_gyro_zout() {
                return (imudata.gyro_zout * 32767 / cfg_gyr_fsr);
        };
        int16_t get_AHRS_yaw() {
                return (imudata.AHRS_yaw * 32767 / 180);
        };
        int16_t get_AHRS_pitch() {
                return (imudata.AHRS_pitch * 32767 / 180);
        };
        int16_t get_AHRS_roll() {
                return (imudata.AHRS_roll * 32767 / 180);
        };
        int16_t get_AHRS_NQuatX() {
                return (imudata.AHRS_NQuatX * 32767);
        };
        int16_t get_AHRS_NQuatY() {
                return (imudata.AHRS_NQuatY * 32767);
        };
        int16_t get_AHRS_NQuatZ() {
                return (imudata.AHRS_NQuatZ * 32767);
        };
    double get_timestamp() const { return imudata.timestamp; }





        float get_accel_xout_f() {
                return imudata.accel_xout;
        };
        float get_accel_yout_f() {
                return imudata.accel_yout;
        };
        float get_accel_zout_f() {
                return imudata.accel_zout;
        };
        float get_gyro_xout_f() {
                return imudata.gyro_xout;
        };
        float get_gyro_yout_f() {
                return imudata.gyro_yout;
        };
        float get_gyro_zout_f() {
                return imudata.gyro_zout;
        };
        float get_AHRS_yaw_f() {
                return imudata.AHRS_yaw;
        };
        float get_AHRS_pitch_f() {
                return imudata.AHRS_pitch;
        };
        float get_AHRS_roll_f() {
                return imudata.AHRS_roll;
        };
        int16_t get_AHRS_NQuatX_f() {
                return imudata.AHRS_NQuatX;
        };
        int16_t get_AHRS_NQuatY_f() {
                return imudata.AHRS_NQuatY;
        };
        int16_t get_AHRS_NQuatZ_f() {
                return imudata.AHRS_NQuatZ;
        };
        float get_mag_xout_f() {
                return imudata.mag_xout;
        };
        float get_mag_yout_f() {
                return imudata.mag_yout;
        };
        float get_mag_zout_f() {
                return imudata.mag_zout;
        };

        // TDK sdk callback
        int  i2c_master_write_register(uint8_t reg, uint32_t len, const uint8_t *data);
        int  i2c_master_read_register(uint8_t reg, uint32_t len, uint8_t *buff);
        void buildSensorEventData(int sensortype, uint64_t timestamp, const void *data, const void *arg);
};
