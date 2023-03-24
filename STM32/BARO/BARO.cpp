#include "BARO.h"
#include <cmath>

Baro::Baro(I2C* i2c) : sensor(i2c) {
    conn_status = BMP_ERROR;
    
    sensor.reset();    
    init_status = sensor.init();
    
    
    this->get_temperature();
    this->get_pressure();
}

Baro::Baro(PinName SDA, PinName SCL) : sensor(SDA, SCL) {
    conn_status = BMP_ERROR;
    
    sensor.reset();
    init_status = sensor.init();

    this->get_temperature();    
    this->get_pressure();
}

int Baro::get_pressure() {
    sensor.startPressure(BMP180::STANDARD);
    
    /* UltaLowPower => 5ms
     * Stadard      => 8ms
     * HighRes      => 14ms
     * UltraHighRes => 26ms
     */
    ThisThread::sleep_for(8ms);
    
    int pressure = 0;
    conn_status = sensor.getPressure(&pressure);


    return pressure;
}

float Baro::get_temperature() {
    sensor.startTemperature();

    ThisThread::sleep_for(5ms);

    float temp = 0;
    conn_status = sensor.getTemperature(&temp);

    return temp;
}

double Baro::get_alt() {
    double T = this->get_temperature();
    if (conn_status != BMP_OK) return -1;
    float P = (float) this->get_pressure();
    if (conn_status != BMP_OK) return -1;

    double alt = this->base_pres / P;
    alt = pow(alt, RATIO1);
    alt -= 1.0;
    alt *= T;
    alt /= RATIO2;
    alt += this->base_alt;

    return alt;
}

uint8_t Baro::configure(double alt, size_t iter) {
    float total = 0.0;

    size_t i = iter;
    while (i > 0) {
        int pt = this->get_pressure();
        if (conn_status == BMP_OK) {
            total += (float) pt;
            i --;
        }
        else
        {
            return BMP_ERROR;
        }
        ThisThread::sleep_for(50ms);
    }

    this->base_alt = alt;
    this->base_pres = total/(float)iter;
    return BMP_OK;
}

uint8_t Baro::get_noise(size_t iter, double* _noise){
    double vals[iter];
    double mean = 0.0;
    size_t i = iter;

    while (i > 0) {
        double temp_alt = this->get_alt();
        if (conn_status == BMP_OK) {
            mean += temp_alt;
            vals[i-1] = temp_alt;
            i--;
        }
        else
        {
            return BMP_ERROR;
            break;
        }
    }

    mean /= (float)iter;

    double stdev = 0;

    for (int i = 0; i<iter; i++) {
        stdev += (mean-vals[i]) * (mean-vals[i]);
    }

    stdev /= (float)iter;
    *_noise = sqrt(stdev);

    return BMP_OK;
}

bool Baro::isReady() {
    return sensor.init() == BMP_OK;
}