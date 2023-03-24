#include "mbed.h"
#include "SerialStream.h"
#include "IMU.h"
#include "BARO.h"
#include <cstdio>
#include "KalmanFilter.h"
#include "SerialGPS.h"
#include <USBSerial.h>
#include "Radio.h"

#define USB_TX_PIN          USBTX
#define USB_RX_PIN          USBRX

#define ERROR_ITER_ACCEL    200
#define ERROR_ITER_BARO     200

#define SENSOR_INIT_RETRY 5

#define ACCEL_WEIGHT        1.0
#define BARO_WEIGHT         100.0
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
SerialGPS gps(PA_2, PA_3, 9600);

bool imu_init(double* noise) {
    CONSOLE("IMU[INFO]: Setting up IMU\n");
    
    calib state = imu.get_calibration();
    if (!imu.conn_status) {
      CONSOLE("IMU[FAILURE]: Cannot connect to BNO055 please check connection\n");
      return false;

    }
    CONSOLE("IMU[SUCCESS]: sys: %d, acc: %d, gyr: %d, mag: %d\n", state.sys, state.acc, state.gyr, state.mag);
    

    vec3 res = imu.calibrate_static_error(ERROR_ITER_ACCEL);
    if (!imu.conn_status) {
        CONSOLE("IMU[FAILURE]: failed to calibrate static error\n");
        return false;
    }
    CONSOLE("IMU[SUCCESS]: X: %s, Y: %s, Z: %s\n", to_str(res.x).c, to_str(res.y).c, to_str(res.z).c);
    

    *noise = imu.get_noise(ERROR_ITER_ACCEL);
    if (!imu.conn_status) {
        CONSOLE("IMU[FAILURE]: failed to calculate accelerometer noise\n");
        return false;
        
    }

    CONSOLE("IMU[SUCCESS]: noise: %s\n", to_str(*noise).c);
    return true;
        
}

bool baro_init(double* noise) {
    CONSOLE("BARO[INFO]: configuring baromter\n");
    
    if (!baro.configure(INITIAL_ALT, ERROR_ITER_BARO)) {
        CONSOLE("BARO[FAILURE]: failed to calibrate barometer\n");
        return false;
    }
    CONSOLE("BARO[SUCCESS]: calbrated | base pressure: %s\n", to_str(baro.base_pres).c);
    CONSOLE("BARO[INFO]: getting noise\n");


    if (!baro.get_noise(ERROR_ITER_BARO, noise)) {
        CONSOLE("BARO[FAILED]: failed to get barometer noise\n");
        return false;
    }
    CONSOLE("BARO[SUCCESS]: noise: %s\n", to_str(*noise).c);
    return true;
}


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


    double accel_noise = -1;
    for (int i = 0; i<SENSOR_INIT_RETRY; i++) {
        if (imu_init(&accel_noise))
            break;
        CONSOLE("IMU[FAILUE]: failed to fully initialize IMU. Retrying %d / %d", i+1, SENSOR_INIT_RETRY);
    }    
    CONSOLE("\n");

    double baro_noise = -1;
    for (int i = 0; i<SENSOR_INIT_RETRY; i++) {
        if (baro_init(&baro_noise))
            break;
        CONSOLE("BARO[FAILUE]: failed to fully initialize Barometer. Retrying %d / %d", i+1, SENSOR_INIT_RETRY);
    }

    CONSOLE("\n");
    

    // Report Init Result and Self Check
    CONSOLE("---------------\nSelf Check Report:\n");
    if (accel_noise != -1)
        CONSOLE("IMU     [OK]\n");
    else
        CONSOLE("IMU     [FAIL]\n");
    if (baro_noise != -1)
        CONSOLE("BARO    [OK]\n");
    else
        CONSOLE("BARO    [FAIL]\n");
    CONSOLE("---------------\n");
    // end of self check

    // TODO pass status of sensors into KalmanFilter to adjust

    ThisThread::sleep_for(5s);


    // ================== CONFIG FILTER ==================
    KalmanFilter filter(
        INITIAL_ALT,
        accel_noise*ACCEL_WEIGHT,
        baro_noise*BARO_WEIGHT,
        (baro_noise*baro_noise)*BARO_VEL_WEIGHT,
        1.0
    );

    filter.start_async(&imu, &baro, &gps);

    // ==================  
    // if anything not init, filter wont working
    while (true) {

        auto alt = filter.altitude();
        auto vel = filter.velocity();

        gpsData d = filter.last_gps();

        CONSOLE("alt: %s, %s | vel: %s, %s | z: %s balt: %s, dt: %s | alt: %s, vdop: %s, time: %s\n",
            to_str(alt.val).c, to_str(alt.var).c,
            to_str(vel.val).c, to_str(vel.var).c,
            to_str(filter.last_acc()).c, to_str(filter.last_baro_alt()).c, to_str(filter.last_dt()).c,
            to_str(d.alt).c, to_str(d.vdop).c, to_str(d.time).c

        );

        //ThisThread::sleep_for(10ms);
    }
}


//RADIO TEST
/*
int main() {
    Thread t;
    t.start(blink);

    Radio radio(&pc);
    radio.init();
    radio.setup_443();

    radio.set_debug(true);

    size_t i = 0;
    while (true) {
        radio.transmit("test msg", 9);
        pc.printf("sent message\n");
        ThisThread::sleep_for(1s);
    }
}
*/


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
int printd() {
    while (true) {
        CONSOLE(".");
        ThisThread::sleep_for(50ms);
    }
}

int main() {
    Thread b;
    b.start(printd);

    Timer t;
    t.start();
    CONSOLE("starting gps\n");
    while (true) {
        t.reset();
        switch (gps.sample()) {
            case 0: {
                //CONSOLE("%s", gps.msg);
                continue;
            }

            case 1: {
                CONSOLE("\nGGA: lat: %s, lon: %s, alt: %s", to_str(gps.latitude).c, to_str(gps.longitude).c, to_str(gps.alt).c);
                break;
            }

            case 2: {
                CONSOLE("GSA: hdop: %s, vdop: %s, pdop: %s", to_str(gps.hdop).c, to_str(gps.vdop).c, to_str(gps.pdop).c);
                break;
            }
        }

        double dt = t.elapsed_time().count() / 1000000.0;
        CONSOLE(" | dt: %s\n", to_str(dt).c);
    }
}
*/

// Baro Test
/*
int main() {
    Thread t;
    t.start(blink);

    ThisThread::sleep_for(5s);

    CONSOLE("CONFIG: %d\n", baro.configure(48, 200));

    while (true) {
        double alt = baro.get_alt();
        float pres = baro.get_pressure();
        float temp = baro.get_temperature();
        CONSOLE("alt: %s | pres: %s | temp: %s | %d, %d | balt: %s, bpres: %s | %d.%d\n", to_str(alt).c, to_str(pres).c, to_str(temp).c,
        baro.conn_status, baro.init_status,
        to_str(baro.base_alt).c, to_str(baro.base_pres).c, (int)alt, (int)((alt - (int)alt)*1000)
        );
        ThisThread::sleep_for(50ms);
    }
}
*/

// I2C Scan
/*
int ack;   
int address;  
void scanI2C(I2C* i2c) {
  for(address=1;address<127;address++) {    
    ack = i2c->write(address << 1, "11", 1);
    if (ack == 0) {
       CONSOLE("\tFound at %3d -- 0x%3x\r\n", address,address);
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
  CONSOLE("I2C-2\n");
  scanI2C(&i2c2);
  CONSOLE("Finished Scan\r\n");
}
*/