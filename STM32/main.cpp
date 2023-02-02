#include "mbed.h"
#include "SerialStream.h"
#include "IMU.h"
#include "BARO.h"
#include <cstdio>
#include "KalmanFilter.h"

#define ERROR_ITER_ACCEL 200
#define ERROR_ITER_BARO 200

#define ACCEL_WEIGHT    1.0
#define BARO_WEIGHT     60.0
#define BARO_VEL_WEIGHT 200.0

#define INITIAL_ALT 48.0

BufferedSerial serial(USBTX, USBRX, 115200);
SerialStream<BufferedSerial> pc(serial);


I2C i2c(I2C_SDA, I2C_SCL);   // SDA, SCL
I2C i2c2(PB_14, PB_13);   // SDA, SCL
IMU imu(i2c2);
Baro baro(&i2c);



/*int main() {

    // ================== CONFIG IMU ==================
    pc.printf("getting IMU %s, %s, %s, %s\n", to_str(0.259).c, to_str(0.0259).c, to_str(0.00259).c, to_str(0.000259).c);
    
    calib state = imu.get_calibration();
    pc.printf("sys: %d, acc: %d, gyr: %d, mag: %d\n", state.sys, state.acc, state.gyr, state.mag);

    vec3 res = imu.calibrate_static_error(100);
    pc.printf("X: %s, Y: %s, Z: %s\n", to_str(res.x).c, to_str(res.y).c, to_str(res.z).c);

    double accel_noise = imu.get_noise(ERROR_ITER_ACCEL);
    pc.printf("noise: %s\n", to_str(accel_noise).c);

    // ==================

    // ================== CONFIG BARO ==================

    pc.printf("configuring baromter\n");
    baro.configure(INITIAL_ALT, 150);

    pc.printf("getting noise\n");
    double baro_noise = baro.get_noise(ERROR_ITER_BARO);

    pc.printf("noise: %s\n", to_str(baro_noise).c);

    // ==================

    // ================== CONFIG FILTER ==================
        KalmanFilter filter(
            INITIAL_ALT,
            accel_noise*ACCEL_WEIGHT,
            baro_noise*BARO_WEIGHT,
            BARO_VEL_WEIGHT*(baro_noise*baro_noise),
            1.0
        );


    // ==================   

    while (true) {
        filter.tick(&imu, &baro);

        auto alt = filter.altitude();
        auto vel = filter.velocity();
        pc.printf("alt: %s, %s | vel: %s, %s | z: %s balt: %s, dt: %s\n",
            to_str(alt.val).c, to_str(alt.var).c,
            to_str(vel.val).c, to_str(vel.var).c,
            to_str(filter.last_acc).c, to_str(filter.last_baro).c, to_str(filter.last_dt()).c
        );

        ThisThread::sleep_for(20ms);
    } 
    
    



}*/


int main() {
    // ================== CONFIG IMU ==================
    pc.printf("getting IMU %s, %s, %s, %s\n", to_str(0.259).c, to_str(0.0259).c, to_str(0.00259).c, to_str(0.000259).c);
    
    calib state = imu.get_calibration();
    pc.printf("sys: %d, acc: %d, gyr: %d, mag: %d\n", state.sys, state.acc, state.gyr, state.mag);

    vec3 res = imu.calibrate_static_error(100);
    pc.printf("X: %s, Y: %s, Z: %s\n", to_str(res.x).c, to_str(res.y).c, to_str(res.z).c);

    double accel_noise = imu.get_noise(ERROR_ITER_ACCEL);
    pc.printf("noise: %s\n", to_str(accel_noise).c);

    // ==================

    // ================== CONFIG BARO ==================

    pc.printf("configuring baromter\n");
    baro.configure(INITIAL_ALT, 150);

    pc.printf("getting noise\n");
    double baro_noise = baro.get_noise(ERROR_ITER_BARO);

    pc.printf("noise: %s\n", to_str(baro_noise).c);

    // ==================

    // ================== CONFIG FILTER ==================
        KalmanFilter filter(
            INITIAL_ALT,
            accel_noise*ACCEL_WEIGHT,
            baro_noise*BARO_WEIGHT,
            BARO_VEL_WEIGHT*(baro_noise*baro_noise),
            1.0
        );

        filter.start_async(&imu, &baro);

    // ==================  

    size_t i = 0;
    while (true) {
        auto alt = filter.altitude();
        auto vel = filter.velocity();
        pc.printf("alt: %s, %s | vel: %s, %s | z: %s balt: %s, dt: %s\n",
            to_str(alt.val).c, to_str(alt.var).c,
            to_str(vel.val).c, to_str(vel.var).c,
            to_str(filter.last_acc).c, to_str(filter.last_baro).c, to_str(filter.last_dt()).c
        );

        ThisThread::sleep_for(10ms);
        i ++;
    }
}


/*int main() {
    // ================== CONFIG IMU ==================
    pc.printf("getting IMU\n");
    
    calib state = imu.get_calibration();
    pc.printf("sys: %d, acc: %d, gyr: %d, mag: %d\n", state.sys, state.acc, state.gyr, state.mag);

    vec3 res = imu.calibrate_static_error(300);
    pc.printf("X: %s, Y: %s, Z: %s\n", to_str(res.x).c, to_str(res.y).c, to_str(res.z).c);

    // ==================

    while (true) {
        auto acc = imu.accel();

        pc.printf("X: %s, Y: %s, Z: %s\n", to_str(acc.x).c, to_str(acc.y).c, to_str(acc.z).c);

        ThisThread::sleep_for(30ms);
    }
}*/