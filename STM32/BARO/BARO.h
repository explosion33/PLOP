#include "mbed.h"
#include "BMP180.h"
#include "SerialStream.h"

#define match(a, b) a.val; if(!a.has) {return Option<b>();}

#define RATIO1 0.19022256
#define RATIO2 0.0065

template <typename T>
struct Option {
    T val;
    bool has;

    Option() : has(false) {}
    Option(const T& value) : has(true), val(value) {}
};

class Baro {
public:
    Baro(I2C* i2c);
    Baro(PinName SDA, PinName SCL);

    Option<int> get_pressure();
    Option<float> get_temperature();
    Option<double> get_alt();

    void configure(double alt, size_t iter);

    double get_noise(size_t iter);

    bool isReady();

private:
    BMP180 sensor;

    int base_alt;
    float base_pres;
};