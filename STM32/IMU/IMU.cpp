#include "IMU.h"
#include "mbed.h"
#include <cmath>
#include <cstdint>
#include <string>

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
    this->conn_status = this->sensor.get_quaternion(&this->quat_storage);

    //double scale = ((double)(1 << 14));

    quat res;
    res.w = ((double)this->quat_storage.w) / 16384.0;
    res.x = (double)(this->quat_storage.x) / 16384.0;
    res.y = (double)(this->quat_storage.y) / 16384.0;
    res.z = (double)(this->quat_storage.z) / 16384.0;

    return res;
}

vec3 IMU::accel() {
    this->conn_status = this->sensor.get_linear_accel(&this->accel_storage);

    vec3 res;
    res.x = this->accel_storage.x;
    res.y = this->accel_storage.y;
    res.z = this->accel_storage.z;

    res = this->rotate(res, this->quaternion());

    res.x -= this->errors.x;
    res.y -= this->errors.y;
    res.z -= this->errors.z;

    return res;
}

vec3 IMU::cross(vec3 a, vec3 b) {
    vec3 res;

    res.x = (a.y*b.z) - (a.z*b.y);
    res.y = (a.z*b.x) - (a.x*b.z);
    res.z = (a.x*b.y) - (a.y*b.x);

    return res;
}

vec3 IMU::rotate(vec3 vec, quat rot) {
    vec3 qv;
    qv.x = rot.x;
    qv.y = rot.y;
    qv.z = rot.z;

    vec3 c1 = cross(vec, qv);

    vec3 temp;
    temp.x = (rot.w * vec.x) + c1.x;
    temp.y = (rot.w * vec.y) + c1.y;
    temp.z = (rot.w * vec.z) + c1.z;

    vec3 c2 = cross(temp, qv);

    vec3 res;
    res.x = vec.x + (2.0 * c2.x);
    res.y = vec.y + (2.0 * c2.y);
    res.z = vec.z + (2.0 * c2.z);
    
    return res;
}

vec3 IMU::calibrate_static_error(size_t iter) {
    this->errors.x = 0.0;
    this->errors.y = 0.0;
    this->errors.z = 0.0;

    vec3 totals;
    totals.x = 0.0;
    totals.y = 0.0;
    totals.z = 0.0;

    for (int i = 0; i<iter; i++) {
        vec3 res = this->accel();

        totals.x += res.x;
        totals.y += res.y;
        totals.z += res.z;

        ThisThread::sleep_for(30ms);
    }

    totals.x /= (double)iter;
    totals.y /= (double)iter;
    totals.z /= (double)iter;

    this->errors = totals;

    return this->errors;
}

calib IMU::get_calibration(uint8_t *_state) {
    uint8_t reg = this->sensor.read_calib_status(_state);

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