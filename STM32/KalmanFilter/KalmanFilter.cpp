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


KalmanFilter::KalmanFilter(double alt, double accel_weight, double baro_weight, double baro_vel_weight, double gps_weight)
    : vel(0.0, 0.0), alt(alt, 0.0) {

        this->complete = false;
        this->last_baro_alt = alt;

        this->ACCEL_WEIGHT = accel_weight;
        this->BARO_WEIGHT = baro_weight;
        this->BARO_VEL_WEIGHT = baro_vel_weight;
        this->GPS_WEIGHT = gps_weight;

        for (int i = 0; i<BARO_ROLLING_AVERAGE; i++) {
            this->baros[i] = 0.0;
        }

        t.start();
}

double KalmanFilter::tick_accel(IMU* imu) {

    double res = imu->accel().z;
    this->last_acc = res;
    return res;
}

double KalmanFilter::tick_baro(Baro* baro) {
    double res = 0.0;

    auto at = baro->get_alt();
    if (at.has) {
        for (int i = 1; i<BARO_ROLLING_AVERAGE; i++) {
            this->baros[i] = this->baros[i-1];
            res += this->baros[i];
        }
        this->baros[0] = at.val;
        res += at.val;
    }
    else {
        for (int i = 0; i<BARO_ROLLING_AVERAGE; i++) {
            res += this->baros[i];
        }
    }

    res /= (double)(BARO_ROLLING_AVERAGE);
    
    this->last_baro = res;
    return res;

}

void KalmanFilter::tick(IMU* imu, Baro* baro) {
    this->dt = this->t.elapsed_time().count() / 1000000.0; // us to s
    this->t.reset();

    double acc = this->tick_accel(imu);
    this->alt.predict(this->vel.val*dt + 0.5*acc*(dt*dt), (this->ACCEL_WEIGHT * dt) + this->vel.var*dt);
    this->vel.predict(acc*dt, this->ACCEL_WEIGHT*dt);

    double balt = this->tick_baro(baro);
    this->alt.update(balt, this->BARO_WEIGHT*dt);
    this->vel.update((balt - this->last_baro_alt)*dt, this->BARO_VEL_WEIGHT*dt);
    this->last_baro_alt = balt;
}

void do_accel_async(async* data) {
    Timer t;
    t.start();
    while (data->running) {
        double dt = t.elapsed_time().count() / 1000000.0;
        t.reset();

        double acc = data->imu->accel().z;
        data->alt->predict(data->vel->val*dt + 0.5*acc*(dt*dt), (data->ACCEL_WEIGHT * dt) + data->vel->var*dt);
        data->vel->predict(acc*dt, data->ACCEL_WEIGHT*dt);

        *data->last_acc = acc;
    }
}

void do_baro_async(async* data) {
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
        
        data->alt->update(res, data->BARO_WEIGHT*dt);
        data->vel->update((res - last_baro_alt)*dt, data->BARO_VEL_WEIGHT*dt);
        last_baro_alt = res;

        *data->last_baro = res;
    }
}

void KalmanFilter::start_async(IMU* imu, Baro* baro) {
    this->running = true;
    
    this->adata = {
        &this->alt,
        &this->vel,
        imu,
        baro,
        &this->running,
        this->ACCEL_WEIGHT,
        this->BARO_WEIGHT,
        this->BARO_VEL_WEIGHT,
        this->GPS_WEIGHT,
        &this->last_acc,
        &this->last_baro,
    };

    t1.start(callback(do_accel_async, &adata));
    t2.start(callback(do_baro_async, &adata));
}

void KalmanFilter::stop_async() {
    this->running = false;
    t1.terminate();
    t2.terminate();
    t1.join();
    t2.join();
}

data KalmanFilter::altitude() {
    data res;
    res.val = this->alt.val;
    res.var = this->alt.var;
    return res;
}

data KalmanFilter::velocity() {
    data res;
    res.val = this->vel.val;
    res.var = this->vel.var;
    return res;
}

double KalmanFilter::last_dt() {
    return dt;
}