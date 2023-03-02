#ifndef BARO_H
#define BARO_H

#include "mbed.h"
#include "BMP180.h"
#include "Option.h"

#define BARO_WORKING 2
#define BARO_CONFIGD 1
#define BARO_ERROR 0

#define RATIO1 0.19022256
#define RATIO2 0.0065

class Baro {
public:
    Baro(I2C* i2c);
    Baro(PinName SDA, PinName SCL);

    int get_pressure();
    float get_temperature();
    double get_alt();

    uint8_t configure(double alt, size_t iter);

    uint8_t get_noise(size_t iter, double* _noise);

    bool isReady();

    uint8_t conn_status;
    uint8_t init_status; // 0 for failed, 1 for configured, 2 for working

private:
    BMP180 sensor;

    int base_alt;
    float base_pres;
};

#endif