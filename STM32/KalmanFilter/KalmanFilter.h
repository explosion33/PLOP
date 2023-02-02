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

struct async {
    Filter* alt;
    Filter* vel;
    IMU* imu;
    Baro* baro;
    bool* running;

    double ACCEL_WEIGHT;
    double BARO_WEIGHT;
    double BARO_VEL_WEIGHT;
    double GPS_WEIGHT;

    double* last_acc;
    double* last_baro;
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

    double dt;

    double tick_accel(IMU* imu);
    double tick_baro(Baro* baro);

    bool running;
    async adata;

    Thread t1, t2, t3;

public:
    double last_acc, last_baro;

    KalmanFilter(double alt, double accel_weight, double baro_weight, double baro_vel_weight, double gps_weight);
    void tick(IMU* imu, Baro* baro);

    void start_async(IMU* imu, Baro* baro);
    void stop_async();

    data altitude();
    data velocity();
    double last_dt();
};