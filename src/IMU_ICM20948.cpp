#include <stdint.h>
#include "I2C.h"
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <chrono>
#include <thread>
#include "IMU_ICM20948.h"

extern "C" {
#include <Icm20948.h>
#include <SensorTypes.h>
#include <Icm20948MPUFifoControl.h>
#include <Icm20948DataBaseControl.h>

// C stuff for TDK sdk
int      idd_io_hal_read_reg(void *context, uint8_t reg, uint8_t *rbuffer, uint32_t rlen);
int      idd_io_hal_write_reg(void *context, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen);
void     build_sensor_event_data(void *context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void *data, const void *arg);
void     inv_icm20948_sleep(int ms);
void     inv_icm20948_sleep_us(int us);
uint64_t inv_icm20948_get_time_us(void);
};


//---------------------------------------------------------------------
void inv_icm20948_sleep(int ms) {
        usleep(ms * 1000);
}
//---------------------------------------------------------------------
void inv_icm20948_sleep_us(int us) {
        usleep(us);
}
//---------------------------------------------------------------------
uint64_t inv_icm20948_get_time_us(void) {
        uint64_t        us;
        struct timespec spec;

        clock_gettime(CLOCK_REALTIME, &spec);
        us = (spec.tv_sec * 1000 * 1000) + (spec.tv_nsec / 1000);
        return us;
}
//---------------------------------------------------------------------
int idd_io_hal_read_reg(void *context, uint8_t reg, uint8_t *rbuffer, uint32_t rlen) {
        return ((IMU *)context)->i2c_master_read_register(reg, rlen, rbuffer);
}
//---------------------------------------------------------------------
int idd_io_hal_write_reg(void *context, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen) {
        return ((IMU *)context)->i2c_master_write_register(reg, wlen, wbuffer);
}
//---------------------------------------------------------------------
void build_sensor_event_data(void *context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void *data, const void *arg) {
        ((IMU *)context)->buildSensorEventData(sensortype, timestamp, data, arg);
}

static uint8_t                  convert_to_generic_ids[INV_ICM20948_SENSOR_MAX] = {INV_SENSOR_TYPE_ACCELEROMETER,
                                                                                   INV_SENSOR_TYPE_GYROSCOPE,
                                                                                   INV_SENSOR_TYPE_RAW_ACCELEROMETER,
                                                                                   INV_SENSOR_TYPE_RAW_GYROSCOPE,
                                                                                   INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
                                                                                   INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
                                                                                   INV_SENSOR_TYPE_BAC,
                                                                                   INV_SENSOR_TYPE_STEP_DETECTOR,
                                                                                   INV_SENSOR_TYPE_STEP_COUNTER,
                                                                                   INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
                                                                                   INV_SENSOR_TYPE_ROTATION_VECTOR,
                                                                                   INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
                                                                                   INV_SENSOR_TYPE_MAGNETOMETER,
                                                                                   INV_SENSOR_TYPE_SMD,
                                                                                   INV_SENSOR_TYPE_PICK_UP_GESTURE,
                                                                                   INV_SENSOR_TYPE_TILT_DETECTOR,
                                                                                   INV_SENSOR_TYPE_GRAVITY,
                                                                                   INV_SENSOR_TYPE_LINEAR_ACCELERATION,
                                                                                   INV_SENSOR_TYPE_ORIENTATION,
                                                                                   INV_SENSOR_TYPE_B2S};

static enum inv_icm20948_sensor idd_sensortype_conversion(int sensor) {
        switch (sensor) {
                case INV_SENSOR_TYPE_RAW_ACCELEROMETER : return INV_ICM20948_SENSOR_RAW_ACCELEROMETER;
                case INV_SENSOR_TYPE_RAW_GYROSCOPE : return INV_ICM20948_SENSOR_RAW_GYROSCOPE;
                case INV_SENSOR_TYPE_ACCELEROMETER : return INV_ICM20948_SENSOR_ACCELEROMETER;
                case INV_SENSOR_TYPE_GYROSCOPE : return INV_ICM20948_SENSOR_GYROSCOPE;
                case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER : return INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
                case INV_SENSOR_TYPE_UNCAL_GYROSCOPE : return INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED;
                case INV_SENSOR_TYPE_BAC : return INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON;
                case INV_SENSOR_TYPE_STEP_DETECTOR : return INV_ICM20948_SENSOR_STEP_DETECTOR;
                case INV_SENSOR_TYPE_STEP_COUNTER : return INV_ICM20948_SENSOR_STEP_COUNTER;
                case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR : return INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR;
                case INV_SENSOR_TYPE_ROTATION_VECTOR : return INV_ICM20948_SENSOR_ROTATION_VECTOR;
                case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR : return INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
                case INV_SENSOR_TYPE_MAGNETOMETER : return INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD;
                case INV_SENSOR_TYPE_SMD : return INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION;
                case INV_SENSOR_TYPE_PICK_UP_GESTURE : return INV_ICM20948_SENSOR_FLIP_PICKUP;
                case INV_SENSOR_TYPE_TILT_DETECTOR : return INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR;
                case INV_SENSOR_TYPE_GRAVITY : return INV_ICM20948_SENSOR_GRAVITY;
                case INV_SENSOR_TYPE_LINEAR_ACCELERATION : return INV_ICM20948_SENSOR_LINEAR_ACCELERATION;
                case INV_SENSOR_TYPE_ORIENTATION : return INV_ICM20948_SENSOR_ORIENTATION;
                case INV_SENSOR_TYPE_B2S : return INV_ICM20948_SENSOR_B2S;
                default : return INV_ICM20948_SENSOR_MAX;
        } // switch
} // enum sensortyp_conversion

static const uint8_t dmp3_image[] = {
#include "icm20948_img.dmp3a.h"
};

#define AK0991x_DEFAULT_I2C_ADDR 0x0C


int IMU::i2c_master_write_register(uint8_t reg, uint32_t len, const uint8_t *data) {
        if (len == 1) {
                // printf("write 8 at %02x -> %02x\n", reg, data[0]);
                int res = I2CWriteReg8(i2cFd, reg, data[0]);
                if (res < 0) return -1;
        } else if (len == 2) {
                // printf("write 16 at %02x -> %02x %02x\n", reg, data[0], data[1]);
                int res = I2CWriteReg16(i2cFd, reg, data[0] + (data[1] << 8));
                if (res < 0) return -1;
        } else {
                int res = I2CWriteBlock(i2cFd, reg, data, len);
                if (res < 0) return -1;
        }
        return 0;
}

int IMU::i2c_master_read_register(uint8_t reg, uint32_t len, uint8_t *buff) {
        if (len == 1) {
                int res = I2CReadReg8(i2cFd, reg);
                // printf("read 8 at %02x -> %02x\n", reg, res);
                if (res < 0) return -1;
                buff[0] = res & 0xFF;
        } else if (len == 2) {
                int res = I2CReadReg16(i2cFd, reg);
                if (res < 0) return -1;
                buff[0] = res & 0xFF;
                buff[1] = (res >> 8) & 0xFF;
                // printf("read 16 at %02x -> %02x %02x\n", reg, buff[0],buff[1]);
        } else {
                int res = I2CReadBlock(i2cFd, reg, buff, len);
                if (res != len) return -1;
        }
        return 0;
}

void IMU::icm20948_apply_mounting_matrix(void) {
        int ii;
        for (ii = 0; ii < INV_ICM20948_SENSOR_MAX; ii++) {
                inv_icm20948_set_matrix((inv_icm20948_t *)icm_device, cfg_mounting_matrix, (inv_icm20948_sensor)ii);
        }
}

void IMU::icm20948_set_fsr(void) {
        inv_icm20948_set_fsr((inv_icm20948_t *)icm_device, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void *)&cfg_acc_fsr);
        inv_icm20948_set_fsr((inv_icm20948_t *)icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, (const void *)&cfg_acc_fsr);
        inv_icm20948_set_fsr((inv_icm20948_t *)icm_device, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&cfg_gyr_fsr);
        inv_icm20948_set_fsr((inv_icm20948_t *)icm_device, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&cfg_gyr_fsr);
        inv_icm20948_set_fsr((inv_icm20948_t *)icm_device, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void *)&cfg_gyr_fsr);
}

int IMU::icm20948_sensor_setup(void) {
        int     rc;
        uint8_t i, whoami = 0xFF;

        /*
         * Just get the whoami
         */
        rc = inv_icm20948_get_whoami((inv_icm20948_t *)icm_device, &whoami);
        printf("whoami = %d\n", whoami);

        sleep(1);

        /* Setup accel and gyro mounting matrix and associated angle for current board */
        inv_icm20948_init_matrix((inv_icm20948_t *)icm_device);

        printf("dmp image size = %d\n", sizeof(dmp3_image));
        rc = inv_icm20948_initialize((inv_icm20948_t *)icm_device, dmp3_image, sizeof(dmp3_image));
        if (rc != 0) {
                printf("init fail\n");
                return rc;
        }

        /* Initialize auxiliary sensors */
        inv_icm20948_register_aux_compass((inv_icm20948_t *)icm_device,
                                          INV_ICM20948_COMPASS_ID_AK09916,
                                          AK0991x_DEFAULT_I2C_ADDR); // AK0991x_SECONDARY_I2C_ADDR); // AK0991x_DEFAULT_I2C_ADDR);

        rc = inv_icm20948_initialize_auxiliary((inv_icm20948_t *)icm_device);

        if (rc != 0) {
                printf("compass not detected got %d\n", rc);
        } else {
                printf("compass detected\n");
        }
        icm20948_apply_mounting_matrix();
        icm20948_set_fsr();


        /* re-initialize base state structure */
        inv_icm20948_init_structure((inv_icm20948_t *)icm_device);
        return 0;
} // sensor_setup

void IMU::setup() {
        i2cFd = I2CSetupInterface("/dev/i2c-1", I2C_ADDR);

        struct inv_icm20948_serif icm20948_serif;
        icm20948_serif.context                                      = this; /* no need */
        icm20948_serif.read_reg                                     = idd_io_hal_read_reg;
        icm20948_serif.write_reg                                    = idd_io_hal_write_reg;
        icm20948_serif.max_read                                     = 1024 * 16; /* maximum number of bytes allowed per serial read */
        icm20948_serif.max_write                                    = 1024 * 16; /* maximum number of bytes allowed per serial write */

        icm20948_serif.is_spi                                       = false;
        ((inv_icm20948_t *)icm_device)->base_state.serial_interface = SERIAL_INTERFACE_I2C;
        inv_icm20948_reset_states((inv_icm20948_t *)icm_device, &icm20948_serif);
        inv_icm20948_register_aux_compass((inv_icm20948_t *)icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);
        int rc = icm20948_sensor_setup();

        if (rc != 0) {
                printf("unable to init IMU, exiting...\n");
                exit(20);
        }


        if (((inv_icm20948_t *)icm_device)->selftest_done && !((inv_icm20948_t *)icm_device)->offset_done) {
                // If we've run selftes and not already set the offset.
                inv_icm20948_set_offset((inv_icm20948_t *)icm_device, unscaled_bias);
                ((inv_icm20948_t *)icm_device)->offset_done = 1;
        }

        // enable sensors
        rc |= inv_icm20948_enable_sensor((inv_icm20948_t *)icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE), 1);
        rc |= inv_icm20948_enable_sensor((inv_icm20948_t *)icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER), 1); //
                                                                                                                                     // #ifdef IMU_HAVE_9AXIS
        rc |= inv_icm20948_enable_sensor((inv_icm20948_t *)icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_UNCAL_MAGNETOMETER), 1);
        // #endif
        rc |= inv_icm20948_enable_sensor((inv_icm20948_t *)icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR), 1);
        // rc |= inv_icm20948_enable_sensor((inv_icm20948_t*)icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR), 1);

        // set speed to 200Hz
        rc |= inv_icm20948_set_sensor_period((inv_icm20948_t *)icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE), 5);     // 5ms ->200Hz
        rc |= inv_icm20948_set_sensor_period((inv_icm20948_t *)icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER), 5); // 5ms ->200Hz
        // #ifdef IMU_HAVE_9AXIS
        rc |= inv_icm20948_set_sensor_period((inv_icm20948_t *)icm_device,
                                             idd_sensortype_conversion(INV_SENSOR_TYPE_UNCAL_MAGNETOMETER),
                                             5); // 5ms ->200Hz NB: MAGNETO run at 60Hz, it will only report once in a while, when data are ready
        // #endif
        rc |= inv_icm20948_set_sensor_period((inv_icm20948_t *)icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR), 5); // 5ms ->200Hz
        // rc |= inv_icm20948_set_sensor_period((inv_icm20948_t*)icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR), 5);
}

uint8_t IMU::icm20948_get_grv_accuracy(void) {
        uint8_t accel_accuracy;
        uint8_t gyro_accuracy;

        accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
        gyro_accuracy  = (uint8_t)inv_icm20948_get_gyro_accuracy();
        return (min(accel_accuracy, gyro_accuracy));
}

void IMU::ToEulerAngles(float *q, float *a) {
        // q must be normalised
        a[0] = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * 180.0f / M_PI; // yaw
        a[1] = -asin(2.0f * (q[1] * q[3] - q[0] * q[2])) * 180.0f / M_PI;                                                        // pitch
        a[2] = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * 180.0f / M_PI; // roll

        /*


            // roll (x-axis rotation)
            double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
            double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
            a[2] = atan2(sinr_cosp, cosr_cosp) * 180.0f / M_PI;

            // pitch (y-axis rotation)
            double sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
            if (fabs(sinp) >= 1)
                a[1] = copysign(M_PI / 2, sinp) * 180.0f / M_PI; // use 90 degrees if out of range
            else
                a[1] = asin(sinp) * 180.0f / M_PI;

            // yaw (z-axis rotation)
            double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
            double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
            a[0] = atan2(siny_cosp, cosy_cosp) * 180.0f / M_PI;
                */
}


void IMU::buildSensorEventData(int sensortype, uint64_t timestamp, const void *data, const void *arg) {
        float   quat[4];
        uint8_t sensor_id = convert_to_generic_ids[sensortype];
        switch (sensor_id) {
                case INV_SENSOR_TYPE_GYROSCOPE :
                        memcpy(&imudata.gyro_xout, data, sizeof(float) * 3);
                        // memcpy(&(accuracy_flag), arg, sizeof(uint8_t));
                        break;
                case INV_SENSOR_TYPE_ACCELEROMETER :
                        memcpy(&imudata.accel_xout, data, sizeof(float) * 3);
                        // memcpy(&(accuracy_flag), arg, sizeof(uint8_t));
                        break;
                case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER :
                        memcpy(&imudata.mag_xout, data, sizeof(float) * 3);
                        // memcpy(&(accuracy_flag), arg, sizeof(uint8_t));
                        break;
                case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR :
                        memcpy(quat, data, sizeof(float) * 4);
                        // normalise quaternion, just to be sure
                        {
                                float norm = sqrt(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
                                quat[0]    = quat[0] / norm;
                                quat[1]    = quat[1] / norm;
                                quat[2]    = quat[2] / norm;
                                quat[3]    = quat[3] / norm;
                        }
                        imudata.AHRS_NQuatX = quat[1];
                        imudata.AHRS_NQuatY = quat[2];
                        imudata.AHRS_NQuatZ = quat[3];
                        ToEulerAngles(quat, &imudata.AHRS_yaw);
                        // FIXME ?
                        //  This is the timestamp from the fastest sensor integrated by the dmp: should be GYRO & ACCELERO as they are synchronous
                        //  MAGNETO  is slower, do we need a timestamp for it?
                        imudata.timestamp = double(timestamp) / 1000000.0;
                        // accuracy_flag = icm20948_get_grv_accuracy();
                        break;
                default : printf("WARNING: Unsuported sensor data type: %d\n", sensor_id); break;
        }

        /*
        //DEBUG print sometimes
        if(sensor_id == INV_SENSOR_TYPE_GAME_ROTATION_VECTOR)
        {
                static int f = 0;
                static double lts = 0;
                static int reportEvery = 100;
                f++;
                if(f == reportEvery)
                {

                        printf(	"(%.1f fps) %.3f -> A[%8.3f,%8.3f,%8.3f] G[%8.3f,%8.3f,%8.3f] YPR[%9.3f,%9.3f,%9.3f] M[%9.3f,%9.3f,%9.3f]\n",
                                        reportEvery / (imudata.timestamp - lts) ,
                                        imudata.timestamp	    				,
                                        imudata.accel_xout      				,
                                        imudata.accel_yout      				,
                                        imudata.accel_zout      				,
                                        imudata.gyro_xout       				,
                                        imudata.gyro_yout       				,
                                        imudata.gyro_zout       				,
                                        imudata.AHRS_yaw        				,
                                        imudata.AHRS_pitch      				,
                                        imudata.AHRS_roll 	    				,
                                        imudata.mag_xout  	    				,
                                        imudata.mag_yout  	    				,
                                        imudata.mag_zout 						);

                                        //printf(	" YPR[%9.3f,%9.3f,%9.3f]\n",
                                        //imudata.AHRS_yaw        				,
                                        //imudata.AHRS_pitch      				,
                                        //imudata.AHRS_roll 	    				);


                        f = 0;
                        lts = imudata.timestamp;
                }
        }
*/
}


IMU::IMU() {
        icm_device = new inv_icm20948_t;
        setup();
        tries = 0;
}

IMU::~IMU() {
        delete (inv_icm20948_t *)icm_device;
}

bool IMU::readSensor() {
        memset(&imudata, 0, sizeof(IMUDATA));
        inv_icm20948_poll_sensor((inv_icm20948_t *)icm_device, this, build_sensor_event_data);
        if (imudata.timestamp != 0) {
                tries = 0;
                return true;
        }

        // IMU is locked -> exit?
        tries++;
        if (tries > 2000) {
                printf("####################################\nIMU is dead, exiting...\n####################################\n");
                exit(30);
        }

        return false;
}
