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


struct filterData {
    bool running;
    
    Filter alt;
    Filter vel;

    IMU* imu;
    Baro* baro;

    double ACCEL_WEIGHT;
    double BARO_WEIGHT;
    double BARO_VEL_WEIGHT;
    double GPS_WEIGHT;

    double last_acc;
    double last_baro;
    double last_dt;

    bool has_new_baro;
    double curr_baro;
    Timer syncTimer;
};


class KalmanFilter {
private:
    filterData async_data;

    Thread thread_imu, thread_baro;

public:
    double last_acc();
    double last_baro_alt();

    KalmanFilter(double alt, double accel_weight, double baro_weight, double baro_vel_weight, double gps_weight);

    void start_async(IMU* imu, Baro* baro);
    void stop_async();

    data altitude();
    data velocity();
    double last_dt();
};