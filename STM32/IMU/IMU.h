#ifndef IMU_H
#define IMU_H

#include "mbed.h"
#include "BNO055.h"
#include <cstddef>
#include <cstdint>
#include "SerialStream.h"

#define IMU_WORKING 3
#define IMU_CALIB_S 2
#define IMU_CALIB 1
#define IMU_ERROR 0

struct vec3 {
    double x, y, z;
};

struct quat {
    double w, x, y, z;
};

struct calib {
    uint8_t sys, gyr, acc, mag;
};

class IMU {
public:
    IMU(I2C& i2c, PinName p_reset);
    IMU(I2C& i2c);

    vec3 euler();
    quat quaternion();
    vec3 accel();

    vec3 calibrate_static_error(size_t iter);
    calib get_calibration(uint8_t *_state);

    double get_noise(size_t iter);

    bool reset();

    uint8_t conn_status; // 1 for working, 0 for connection lost
    uint8_t init_status; // 0 for failed, 1 for configured, 2 for working

private:
    vec3 rotate(vec3 vec, quat rot);

    vec3 cross(vec3 a, vec3 b);

private:
    bool has_reset_pin;

    BNO055 sensor;
    
    BNO055_EULER_TypeDef euler_storage;
    BNO055_QUATERNION_TypeDef quat_storage;
    BNO055_LIN_ACC_TypeDef accel_storage;
	
	vec3 errors;
};

#endif