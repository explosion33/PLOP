#include "IMU.h"
#include "mbed.h"
#include <cmath>
#include <cstdint>

IMU::IMU(I2C& i2c, PinName p_reset) : sensor(i2c, p_reset) {
    this->has_reset_pin = true;
}

IMU::IMU(I2C& i2c) : sensor(i2c, PA_0){
    this->has_reset_pin = false;
}

vec3 IMU::euler() {
    this->sensor.get_Euler_Angles(&this->euler_storage);

    vec3 res;
    res.x = this->euler_storage.h;
    res.y = this->euler_storage.p;
    res.z = this->euler_storage.r;

    return res;
}

quat IMU::quaternion() {
    this->sensor.get_quaternion(&this->quat_storage);

    //double scale = ((double)(1 << 14));

    quat res;
    res.w = ((double)this->quat_storage.w) / 16384.0;
    res.x = (double)(this->quat_storage.x) / 16384.0;
    res.y = (double)(this->quat_storage.y) / 16384.0;
    res.z = (double)(this->quat_storage.z) / 16384.0;

    return res;
}

vec3 IMU::accel() {
    this->sensor.get_linear_accel(&this->accel_storage);

    vec3 res;
    res.x = this->accel_storage.x - this->errors.x;
    res.y = this->accel_storage.y - this->errors.y;
    res.z = this->accel_storage.z - this->errors.z;



    return this->rotate(res, this->quaternion());
}

vec3 IMU::cross(vec3 a, vec3 b) {
    vec3 res;

    res.x = (a.y*b.z) - (a.z*b.y);
    res.y = (a.z*b.x) - (a.x*b.z);
    res.z = (a.x*b.y) - (a.y*b.x);

    return res;
}

quat IMU::mult_quat(quat a, quat b) {
    quat res;

    res.w=(a.w*b.w)-(a.x*b.x)-(a.y*b.y)-(a.z*b.z);
    res.x=(a.x*b.w)+(a.w*b.x)-(a.z*b.y)+(a.y*b.z);
    res.y=(a.y*b.w)+(a.z*b.x)+(a.w*b.y)-(a.x*b.z);
    res.z=(a.z*b.w)-(a.y*b.x)+(a.x*b.y)+(a.w*b.z);

    return res;
}

vec3 IMU::rotate(vec3 vec, quat rot) {
    quat qp = rot;
    qp.x *= -1;
    qp.y *= -1;
    qp.z *= -1;

    quat qv;
    qv.w = 0;
    qv.x = vec.x;
    qv.y = vec.y;
    qv.z = vec.z;

    quat res = mult_quat(mult_quat(qp, qv), rot);

    vec3 out;
    out.x = res.x;
    out.y = res.y;
    out.z = res.z;

    return out;
}

vec3 IMU::calibrate_static_error(size_t iter) {
    this->errors.x = 0;
    this->errors.y = 0;
    this->errors.z = 0;

    for (int i = 0; i<iter; i++) {
        this->sensor.get_linear_accel(&this->accel_storage);

        this->errors.x += this->accel_storage.x;
        this->errors.y += this->accel_storage.y;
        this->errors.z += this->accel_storage.z;

        ThisThread::sleep_for(20ms);
    }

    this->errors.x /= (double)iter;
    this->errors.y /= (double)iter;
    this->errors.z /= (double)iter;

    return this->errors;
}

calib IMU::get_calibration() {
    uint8_t reg = this->sensor.read_calib_status();

    calib res;

    res.sys = (reg >> 6) & 3;
    res.gyr = (reg >> 4) & 3;
    res.acc = (reg >> 2) & 3;
    res.mag = reg & 3;

    return res;
}

double IMU::get_noise(size_t iter) {
    double vals[iter];
    double mean = 0;

    for (int i = 0; i<iter; i++) {
        double res = this->accel().z;
        vals[i] = res;
        mean += res;
        ThisThread::sleep_for(20ms);
    }
    mean /= (double)iter;

    double stdev = 0;

    for (int i = 0; i<iter; i++) {
        stdev += (mean - vals[i]) * (mean - vals[i]);
    }
    stdev /= (double)iter;
    
    return sqrt(stdev);
}

bool IMU::reset() {
    if (this->has_reset_pin)
        this->sensor.reset();
    
    return this->has_reset_pin;

}