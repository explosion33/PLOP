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
    double dt = 0.033;//this->t.elapsed_time().count() / 1000000.0; // us to s
    this->t.reset();

    double acc = this->tick_accel(imu);
    this->alt.predict(this->vel.val*dt + 0.5*acc*(dt*dt), (this->ACCEL_WEIGHT * dt) + this->vel.var*dt);
    this->vel.predict(acc*dt, this->ACCEL_WEIGHT*dt);

    double balt = this->tick_baro(baro);
    this->alt.update(balt, this->BARO_WEIGHT*dt);
    this->vel.update((balt - this->last_baro_alt)*dt, this->BARO_VEL_WEIGHT*dt);
    this->last_baro_alt = balt;
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