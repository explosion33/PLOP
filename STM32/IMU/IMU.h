#include "mbed.h"
#include "BNO055.h"
#include <cstddef>
#include <cstdint>

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
    calib get_calibration();

    double get_noise(size_t iter);

    bool reset();
    

private:
    vec3 rotate(vec3 vec, quat rot);

    vec3 cross(vec3 a, vec3 b);
    quat mult_quat(quat a, quat b);

private:
    bool has_reset_pin;

    BNO055 sensor;
    
    BNO055_EULER_TypeDef euler_storage;
    BNO055_QUATERNION_TypeDef quat_storage;
    BNO055_LIN_ACC_TypeDef accel_storage;
	
	vec3 errors;
};