#include "mbed.h"
#include "SerialStream.h"
#include "IMU.h"
#include "BARO.h"
#include <cstdio>
#include "KalmanFilter.h"
#include "SerialGPS.h"

#define ERROR_ITER_ACCEL 200
#define ERROR_ITER_BARO 200

#define ACCEL_WEIGHT    1.0
#define BARO_WEIGHT     60.0
#define BARO_VEL_WEIGHT 200.0

#define INITIAL_ALT 48.0

#include <USBSerial.h>

USBSerial pc(false);
DigitalOut led(LED1);

/*BufferedSerial serial(USBTX, USBRX, 115200);
SerialStream<BufferedSerial> pc(serial);*/


I2C i2c(PB_7, PB_6); //I2C-1  // SDA, SCL
I2C i2c2(PB_3, PB_10); // I2C-2  // SDA, SCL
IMU imu(i2c);
Baro baro(&i2c2);

//SerialGPS gps(PA_2, PA_3, 9600);

// sync Kalman Filter
/*
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
    
    



}
*/

// async Kalman Filter

void blink() {
    while (true) {
        led = !led;
        ThisThread::sleep_for(50ms);
    }
}


int main() {
    Thread t;
    t.start(blink);

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
        (baro_noise*baro_noise)*BARO_VEL_WEIGHT,
        1.0
    );

    filter.start_async(&imu, &baro);

    // ==================  

    while (true) {

        auto alt = filter.altitude();
        auto vel = filter.velocity();
        pc.printf("alt: %s, %s | vel: %s, %s | z: %s balt: %s, dt: %s\n",
            to_str(alt.val).c, to_str(alt.var).c,
            to_str(vel.val).c, to_str(vel.var).c,
            to_str(filter.last_acc).c, to_str(filter.last_baro).c, to_str(filter.last_dt()).c
        );

        ThisThread::sleep_for(10ms);
    }
}



// IMU accel test
/*
int main() {
    Thread t;
    t.start(blink);
    // ================== CONFIG IMU ==================
    pc.printf("getting IMU\n");
    
    calib state = imu.get_calibration();
    pc.printf("sys: %d, acc: %d, gyr: %d, mag: %d\n", state.sys, state.acc, state.gyr, state.mag);

    vec3 res = imu.calibrate_static_error(300);
    pc.printf("X: %s, Y: %s, Z: %s\n", to_str(res.x).c, to_str(res.y).c, to_str(res.z).c);

    // ==================

    while (true) {
        auto acc = imu.accel();
        auto rot = imu.euler();

        pc.printf("p: %s, r: %s, y: %s  |  X: %s, Y: %s, Z: %s\n",
        to_str(rot.x).c, to_str(rot.y).c, to_str(rot.z).c,
        to_str(acc.x).c, to_str(acc.y).c, to_str(acc.z).c
        );

        ThisThread::sleep_for(100ms);
    }
}
*/


// GPS TEST
/*
int main() {
    Timer t;
    t.start();
    pc.printf("starting gps\n");
    while (true) {
        t.reset();
        switch (gps.sample()) {
            case Error: {
                pc.printf("error\n");
                break;
            }

            case GGA: {
                pc.printf("GGA: lat: %s, lon: %s, alt: %s\n", to_str(gps.latitude).c, to_str(gps.longitude).c, to_str(gps.alt).c);
                break;
            }

            case RMC: {
                pc.printf("TMC: lat: %s, lon: %s, alt: %s, speed: %s, course: %s\n", to_str(gps.latitude).c, to_str(gps.longitude).c, to_str(gps.alt).c, to_str(gps.speed).c, to_str(gps.course).c);
                break;
            }

            case GSA: {
                pc.printf("GSA: hdop: %s, vdop: %s, pdop: %s\n", to_str(gps.hdop).c, to_str(gps.vdop).c, to_str(gps.pdop).c);
                break;
            }
        }

        double dt = t.elapsed_time().count() / 1000000.0;
        pc.printf("dt: %s ", to_str(dt).c);
    }
}*/

/*int ack;   
int address;  
void scanI2C(I2C* i2c) {
  for(address=1;address<127;address++) {    
    ack = i2c->write(address, "11", 1);
    if (ack == 0) {
       pc.printf("\tFound at %3d -- %3x\r\n", address,address);
    }    
    ThisThread::sleep_for(50ms);
  } 
}
 
int main() {
  Thread t;
  t.start(blink);

    ThisThread::sleep_for(5s);

  pc.printf("I2C scanner \r\n");
  scanI2C(&i2c);
  scanI2C(&i2c2);
  pc.printf("Finished Scan\r\n");
}*/