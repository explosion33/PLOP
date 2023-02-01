#include "BARO.h"
#include <cmath>

Baro::Baro(I2C* i2c) : sensor(i2c) {
    sensor.reset();
    sensor.init();
    this->get_pressure();
    this->get_temperature();
}

Baro::Baro(PinName SDA, PinName SCL) : sensor(SDA, SCL) {
    sensor.reset();
    sensor.init();
    
    this->get_pressure();
    this->get_temperature();
}

Option<int> Baro::get_pressure() {
    sensor.startPressure(BMP180::STANDARD);
    
    /* UltaLowPower => 5ms
     * Stadard      => 8ms
     * HighRes      => 14ms
     * UltraHighRes => 26ms
     */
    ThisThread::sleep_for(8ms);
    
    int pressure;
    if (sensor.getPressure(&pressure) != 0) {
        return Option<int>();
    }

    return Option<int>(pressure);
}

Option<float> Baro::get_temperature() {
    sensor.startTemperature();

    ThisThread::sleep_for(5ms);

    float temp;
    if (sensor.getTemperature(&temp) != 0) {
        return Option<float>();
    }

    return Option<float>(temp);
}

Option<double> Baro::get_alt() {
    float P = (float) match(this->get_pressure(), double)
    double T = match(this->get_temperature(), double)


    double alt = this->base_pres / P;
    alt = pow(alt, RATIO1);
    alt -= 1.0;
    alt *= T;
    alt /= RATIO2;
    alt += this->base_alt;

    return Option<double>(alt);
}

void Baro::configure(double alt, size_t iter) {
    float total = 0.0;

    size_t i = iter;
    while (i > 0) {
        auto pt = this->get_pressure();
        if (pt.has) {
            total += (float) pt.val;
            i --;
        }
        ThisThread::sleep_for(20ms);
    }

    this->base_alt = alt;
    this->base_pres = total/(float)iter;
}

double Baro::get_noise(size_t iter) {
    double vals[iter];
    double mean = 0.0;
    size_t i = iter;

    while (i > 0) {
        auto at = this->get_alt();
        if (at.has) {
            mean += at.val;
            vals[i-1] = at.val;
            i--;
        }
    }

    mean /= (float)iter;

    double stdev = 0;

    for (int i = 0; i<iter; i++) {
        stdev += (mean-vals[i]) * (mean-vals[i]);
    }

    stdev /= (float)iter;

    return sqrt(stdev);
}

bool Baro::isReady() {
    return sensor.init() == 0;
}