#include "KalmanFilter.h"
#include <chrono>

Filter::Filter(double mean, double var) {
    this->val = mean;
    this->var = var;
}

void Filter::update(double _val, double _var) {
    this->val = (val*_var + _val*var)/(var + _var);
    this->var = 1.0/( (1.0/var) + (1.0/_var) );
}

void Filter::predict(double _val, double _var) {
    this->val += _val;
    this->var += _var; 
}


KalmanFilter::KalmanFilter(double alt, double accel_weight, double baro_weight, double baro_vel_weight, double gps_weight) : 

    async_data {
        false,
        
        Filter(alt, 0.0),
        Filter(0.0, 0.0),

        nullptr,
        nullptr,

        accel_weight,
        baro_weight,
        baro_vel_weight,
        gps_weight,

        0,
        alt,
        0,

        false,
        alt,
        Timer(),
    }

{}

void do_accel_async(filterData* data) {
    Timer t;
    t.start();
    while (data->running) {
        double dt = t.elapsed_time().count() / 1000000.0;
        t.reset();

        double acc = data->imu->accel().z;
        data->alt.predict(data->vel.val*dt + 0.5*acc*(dt*dt), (data->ACCEL_WEIGHT * dt) + data->vel.var*dt);
        data->vel.predict(acc*dt, data->ACCEL_WEIGHT*dt);

        data->last_acc = acc;
        data->last_dt = dt;

        ThisThread::sleep_for(10ms);
    }
}

void do_baro_async(filterData* data) {
    double baros[BARO_ROLLING_AVERAGE] = {0};
    double last_baro_alt = 0;

    Timer t;
    t.start();
    while (data->running) {
        double dt = t.elapsed_time().count() / 1000000.0;
        t.reset();

        double res = 0.0;

        auto at = data->baro->get_alt();
        if (at.has) {
            for (int i = 1; i<BARO_ROLLING_AVERAGE; i++) {
                baros[i] = baros[i-1];
                res += baros[i];
            }
            baros[0] = at.val;
            res += at.val;
        }
        else {
            for (int i = 0; i<BARO_ROLLING_AVERAGE; i++) {
                res += baros[i];
            }
        }

        res /= (double)(BARO_ROLLING_AVERAGE);
        
        data->alt.update(res, data->BARO_WEIGHT*dt);
        data->vel.update((res - last_baro_alt)*dt, data->BARO_VEL_WEIGHT*dt);
        last_baro_alt = res;

        data->last_baro = res;
    }
}

void KalmanFilter::start_async(IMU* imu, Baro* baro) {
    this->async_data.running = true;
    this->async_data.imu = imu;
    this->async_data.baro = baro;

    thread_imu.start(callback(do_accel_async, &this->async_data));
    thread_baro.start(callback(do_baro_async, &this->async_data));
}

void KalmanFilter::stop_async() {
    this->async_data.running = false;
    thread_imu.terminate();
    thread_baro.terminate();
    thread_imu.join();
    thread_baro.join();
}

data KalmanFilter::altitude() {
    data res;
    res.val = this->async_data.alt.val;
    res.var = this->async_data.alt.var;
    return res;
}

data KalmanFilter::velocity() {
    data res;
    res.val = this->async_data.vel.val;
    res.var = this->async_data.vel.var;
    return res;
}

double KalmanFilter::last_acc() {
    return this->async_data.last_acc;
}

double KalmanFilter::last_baro_alt() {
    return this->async_data.last_baro;
}

double KalmanFilter::last_dt() {
    return this->async_data.last_dt;
}