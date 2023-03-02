#include "mbed.h"
#include "SerialStream.h"
#include "IMU.h"
#include "BARO.h"
#include <cstdio>
#include "KalmanFilter.h"
#include "SerialGPS.h"
#include <USBSerial.h>

#define USB_TX_PIN          USBTX
#define USB_RX_PIN          USBRX

#define ERROR_ITER_ACCEL    200
#define ERROR_ITER_BARO     200

#define ACCEL_WEIGHT        1.0
#define BARO_WEIGHT         60.0
#define BARO_VEL_WEIGHT     200.0

#define INITIAL_ALT         48.0 //replace eventually

// magic for serial output
#ifndef TARGET_NUCLEO_F446RE
#define CONSOLE pc.printf
USBSerial pc(false);
#else
#define CONSOLE printf
// Create a BufferedSerial object to be used by the system I/O retarget code.
static BufferedSerial serial_port(USB_TX_PIN, USB_RX_PIN, 9600);
FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial_port;
}
// end of serial setup
#endif

DigitalOut led(LED1);



I2C i2c(PB_7, PB_6); //I2C-1  // SDA, SCL
I2C i2c2(PB_3, PB_10); // I2C-2  // SDA, SCL
IMU imu(i2c);
Baro baro(&i2c2);
//SerialGPS gps(PA_2, PA_3, 9600);


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

    ThisThread::sleep_for(10s); // allow time to open console before printing major information

    CONSOLE(
        "PLOP Onboard Mbed OS version %d.%d.%d\n",
        MBED_MAJOR_VERSION,
        MBED_MINOR_VERSION,
        MBED_PATCH_VERSION
    );
    // ================== CONFIG IMU ==================

    CONSOLE("Setting up IMU\n");

    uint8_t conn_state = 0;
    uint8_t retry_counter = 0;
    imu.conn_status = 0;
    double accel_noise = 0;
    vec3 res;
    
    calib state = imu.get_calibration(&conn_state);
    if (conn_state)
    {
        CONSOLE("IMU[SUCCESS]: sys: %d, acc: %d, gyr: %d, mag: %d\n", state.sys, state.acc, state.gyr, state.mag);
        imu.init_status = IMU_CALIB;
        while (!imu.conn_status && retry_counter < 5)
        {
            res = imu.calibrate_static_error(100);
            retry_counter += 1;
        }
        if (conn_state)
        {
            CONSOLE("IMU[SUCCESS]: X: %s, Y: %s, Z: %s\n", to_str(res.x).c, to_str(res.y).c, to_str(res.z).c);
            imu.init_status = IMU_CALIB_S;
            imu.conn_status = 0;
            retry_counter = 0;
        }
        else
        {
            CONSOLE("IMU[FAILURE]: failed to get calib; Conn failed\n");
            CONSOLE("IMU[FAILURE]: failed to get noise; Conn failed\n");
        }
        // if failed, conn will be set to 1, jump over the next accel noise
        while (!imu.conn_status && retry_counter < 5)
        {
            accel_noise = imu.get_noise(ERROR_ITER_ACCEL);
            retry_counter += 1;
        }
        if (imu.conn_status)
        {
            imu.init_status = IMU_WORKING;
            CONSOLE("IMU[SUCCESS]: noise: %s\n", to_str(accel_noise).c);
        }
        else
            CONSOLE("IMU[FAILURE]: failed to get noise\n");
    }
    else
        CONSOLE("IMU[FAILURE]: No connection, plz check;\n");

    // ==================

    // ================== CONFIG BARO ==================

    CONSOLE("BARO[INFO]: configuring baromter\n");
    retry_counter = 0;
    double baro_noise = 0;
    while (retry_counter < 5)
    {
        if (baro.configure(INITIAL_ALT, 150))
        {
            retry_counter = 0;
            baro.init_status = BARO_CONFIGD;
            CONSOLE("BARO[SUCCESS]: configured\n");
            CONSOLE("BARO[INFO]: getting noise\n");
            break;
        }
        retry_counter += 1;
    }
    while (retry_counter < 5)
    {
        if (baro.get_noise(ERROR_ITER_BARO, &baro_noise))
        {
            CONSOLE("BARO[SUCCESS]: noise: %s\n", to_str(baro_noise).c);
            baro.init_status = BARO_WORKING;
            retry_counter = 0;
            break;
        }
        retry_counter += 1;
    }
    if (retry_counter > 0)
    {
        CONSOLE("BARO[FAILED]: connection error \n");
    }
    
    // ==================

    // Report Init Result and Self Check
    CONSOLE("-------------\nSelf Check Report:\n");
    if (imu.init_status == IMU_WORKING)
        CONSOLE("IMU     [OK]\n");
    else
        CONSOLE("IMU     [FAIL] %d\n", imu.init_status);
    if (baro.init_status == BARO_WORKING)
        CONSOLE("BARO    [OK]\n");
    else
        CONSOLE("BARO    [FAIL] %d\n", baro.init_status);
    // end of self check

    conn_state = (baro.init_status == BARO_WORKING) && (imu.init_status == IMU_WORKING);

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
    // if anything not init, filter wont working
    while (conn_state) {

        auto alt = filter.altitude();
        auto vel = filter.velocity();
        CONSOLE("alt: %s, %s | vel: %s, %s | z: %s balt: %s, dt: %s\n",
            to_str(alt.val).c, to_str(alt.var).c,
            to_str(vel.val).c, to_str(vel.var).c,
            to_str(filter.last_acc()).c, to_str(filter.last_baro_alt()).c, to_str(filter.last_dt()).c
        );

        //ThisThread::sleep_for(10ms);
    }
}



// IMU accel test
/*
int main() {
    Thread t;
    t.start(blink);
    // ================== CONFIG IMU ==================
    CONSOLE("getting IMU\n");
    
    calib state = imu.get_calibration();
    CONSOLE("sys: %d, acc: %d, gyr: %d, mag: %d\n", state.sys, state.acc, state.gyr, state.mag);

    vec3 res = imu.calibrate_static_error(300);
    CONSOLE("X: %s, Y: %s, Z: %s\n", to_str(res.x).c, to_str(res.y).c, to_str(res.z).c);

    // ==================

    while (true) {
        auto acc = imu.accel();
        auto rot = imu.euler();

        CONSOLE("p: %s, r: %s, y: %s  |  X: %s, Y: %s, Z: %s\n",
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
    CONSOLE("starting gps\n");
    while (true) {
        t.reset();
        switch (gps.sample()) {
            case Error: {
                CONSOLE("error\n");
                break;
            }

            case GGA: {
                CONSOLE("GGA: lat: %s, lon: %s, alt: %s\n", to_str(gps.latitude).c, to_str(gps.longitude).c, to_str(gps.alt).c);
                break;
            }

            case RMC: {
                CONSOLE("TMC: lat: %s, lon: %s, alt: %s, speed: %s, course: %s\n", to_str(gps.latitude).c, to_str(gps.longitude).c, to_str(gps.alt).c, to_str(gps.speed).c, to_str(gps.course).c);
                break;
            }

            case GSA: {
                CONSOLE("GSA: hdop: %s, vdop: %s, pdop: %s\n", to_str(gps.hdop).c, to_str(gps.vdop).c, to_str(gps.pdop).c);
                break;
            }
        }

        double dt = t.elapsed_time().count() / 1000000.0;
        CONSOLE("dt: %s ", to_str(dt).c);
    }
}*/

/*int ack;   
int address;  
void scanI2C(I2C* i2c) {
  for(address=1;address<127;address++) {    
    ack = i2c->write(address, "11", 1);
    if (ack == 0) {
       CONSOLE("\tFound at %3d -- %3x\r\n", address,address);
    }    
    ThisThread::sleep_for(50ms);
  } 
}
 
int main() {
  Thread t;
  t.start(blink);

    ThisThread::sleep_for(5s);

  CONSOLE("I2C scanner \r\n");
  scanI2C(&i2c);
  scanI2C(&i2c2);
  CONSOLE("Finished Scan\r\n");
}*/