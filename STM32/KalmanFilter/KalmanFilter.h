#include "mbed.h"
#include "BARO.h"
#include "IMU.h"
#include "SerialGPS.h"

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

struct gpsData {
    float lat, lon, alt;
    float pdop, hdop, vdop;
    float time;
};


struct filterData {
    bool running;
    
    Filter alt;
    Filter vel;

    IMU* imu;
    Baro* baro;
    SerialGPS* gps;

    double ACCEL_WEIGHT;
    double BARO_WEIGHT;
    double BARO_VEL_WEIGHT;
    double GPS_WEIGHT;

    double last_acc;
    double last_baro;
    double last_dt;

    quat last_quat;

    // high dv baro + IMU desync fix
    bool has_new_baro;
    double curr_baro;
    double baro_vel;
    double baro_dt;
    Timer syncTimer;
};


class KalmanFilter {
private:
    filterData async_data;

    Thread thread_imu, thread_baro, thread_gps;

public:
    double last_acc();
    double last_baro_alt();

    KalmanFilter(double alt, double accel_weight, double baro_weight, double baro_vel_weight, double gps_weight);

    void start_async(IMU* imu, Baro* baro, SerialGPS* gps);
    void stop_async();

    data altitude();
    data velocity();
    double last_dt();
    gpsData last_gps();

    quat last_quat();
};