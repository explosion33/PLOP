#include "mbed.h"
#include "BARO.h"
#include "IMU.h"

#define BARO_ROLLING_AVERAGE 100

class Filter {
public:
    double val, var;

    Filter(double mean, double var);

    void update(double mean, double var);
    void predict(double mean, double var);
};

struct data {
    double val, var;
};

class KalmanFilter {
private:
    Timer t;
    Filter vel, alt;
    
    double last_gps[3] = {0};
    bool complete;

    double last_baro_alt;

    double baros[BARO_ROLLING_AVERAGE] = {0};

    double ACCEL_WEIGHT;
    double BARO_WEIGHT;
    double BARO_VEL_WEIGHT;
    double GPS_WEIGHT;

    double tick_accel(IMU* imu);
    double tick_baro(Baro* baro);

public:
    double last_acc, last_baro;

    KalmanFilter(double alt, double accel_weight, double baro_weight, double baro_vel_weight, double gps_weight);
    void tick(IMU* imu, Baro* baro);

    data altitude();
    data velocity();
};