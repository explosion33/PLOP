#include "mbed.h"
#include "SerialStream.h"
#include "IMU.h"
#include "BARO.h"
#include <cstdio>

BufferedSerial serial(USBTX, USBRX, 115200);
SerialStream<BufferedSerial> pc(serial);


I2C i2c(I2C_SDA, I2C_SCL);   // SDA, SCL
IMU imu(i2c);  // Reset =D7, addr = BNO055_G_CHIP_ADDR, mode = MODE_NDOF <- as default
Baro baro(&i2c);

struct str {
    char c[12];
};

str to_str(double num) {
    str out;

    int l = (int)num;
    int r = abs((num-l)*1000);

    sprintf(out.c, "%+d.%d", l, r);

    bool found = false;
    for (int i = 0; i<12; i++) {
        if (out.c[i] == 0) {
            found = true;
        }
        if (found) {
            out.c[i] = ' ';
        }
    }

    out.c[11] = 0;

    return out;
}

int main()
{

    // ================== CONFIG IMU ==================
    pc.printf("getting IMU\n");
    
    calib state = imu.get_calibration();
    pc.printf("sys: %d, acc: %d, gyr: %d, mag: %d", state.sys, state.acc, state.gyr, state.mag);

    vec3 res = imu.calibrate_static_error(300);
    pc.printf("X: %s, Y: %s, Z: %s\n", to_str(res.x).c, to_str(res.y).c, to_str(res.z).c);

    double error = imu.get_noise(400);
    pc.printf("noise: %s\n", to_str(error).c);

    // ==================

    // ================== CONFIG BARO ==================

    pc.printf("configuring baromter\n");
    baro.configure(18.0, 150);

    pc.printf("getting noise\n");
    double noise = baro.get_noise(150);

    pc.printf("noise: %s\n", to_str(noise).c);

    // ==================
    
    while (true) {
        auto at = baro.get_alt();
        /*if (at.has) {
            pc.printf("alt: %s pres: %d   temp: %s\n", to_str(at.val).c, baro.get_pressure().val, to_str(baro.get_temperature().val).c);
        }*/
        
        
        vec3 angles = imu.euler();
        quat quat_d = imu.quaternion();
        vec3 accels = imu.accel();

        pc.printf("H: %s, R: %s, P: %s  ||  %s %s %s %s  ||  X: %s, Y: %s, Z: %s  ||  %s\n",
            to_str(angles.x).c, to_str(angles.y).c, to_str(angles.z).c,
            to_str(quat_d.w).c, to_str(quat_d.x).c, to_str(quat_d.y).c, to_str(quat_d.z).c,
            to_str(accels.x).c, to_str(accels.y).c, to_str(accels.z).c,
            to_str(at.val).c
        );

        //ThisThread::sleep_for(10ms);
    }

}