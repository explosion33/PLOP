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
        0.0,
        0.0,
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


        if (data->has_new_baro) {
            data->has_new_baro = false;

            // get time since last acceleration
            double dtsa = data->syncTimer.elapsed_time().count() / 1000000.0;

            // preict to halfway point
            data->alt.predict(data->vel.val*dtsa + 0.5*acc*(dtsa*dtsa), (data->ACCEL_WEIGHT * dtsa) + data->vel.var*dtsa);
            data->vel.predict(acc*dtsa, data->ACCEL_WEIGHT*dtsa);

            // update from baro
            data->alt.update(data->curr_baro, data->BARO_WEIGHT*data->baro_dt);
            data->vel.update(data->baro_vel*data->baro_dt, data->BARO_VEL_WEIGHT*data->baro_dt);
            
            // update second half
            dtsa = dt - dtsa;
            data->alt.predict(data->vel.val*dtsa + 0.5*acc*(dtsa*dtsa), (data->ACCEL_WEIGHT * dtsa) + data->vel.var*dtsa);
            data->vel.predict(acc*dtsa, data->ACCEL_WEIGHT*dtsa);

            data->syncTimer.start();
        }

        else {
            data->alt.predict(data->vel.val*dt + 0.5*acc*(dt*dt), (data->ACCEL_WEIGHT * dt) + data->vel.var*dt);
            data->vel.predict(acc*dt, data->ACCEL_WEIGHT*dt);
        }

        data->last_acc = acc;
        data->last_dt = dt;

        data->syncTimer.reset();

        ThisThread::sleep_for(10ms);
    }
}

void do_baro_async(filterData* data) {
    double baros[BARO_ROLLING_AVERAGE] = {0};
    double last_baro_alt = 0;

    Timer t;
    t.start();
    while (data->running) {
        data->syncTimer.stop();

        double dt = t.elapsed_time().count() / 1000000.0;
        t.reset();

        double res = 0.0;

        // get alt and run the average
        double at = data->baro->get_alt();
        if (data->baro->conn_status) {
            for (int i = 1; i<BARO_ROLLING_AVERAGE; i++) {
                baros[i] = baros[i-1];
                res += baros[i];
            }
            baros[0] = at;
            res += at;
        }
        else {
            for (int i = 0; i<BARO_ROLLING_AVERAGE; i++) {
                res += baros[i];
            }
        }
        res /= (double)(BARO_ROLLING_AVERAGE);
        
        // moved to acc thread
        /*
        data->alt.update(res, data->BARO_WEIGHT*dt);
        data->vel.update((res - last_baro_alt)*dt, data->BARO_VEL_WEIGHT*dt);
        last_baro_alt = res;
        */

        //store data for future update
        data->has_new_baro = true;
        data->curr_baro = res;
        data->baro_vel = (res - last_baro_alt)*dt;
        data->baro_dt = dt;

        last_baro_alt = res;

        data->last_baro = res;
    }
}

void do_gps_async(filterData* data) {
    Timer t;
    t.start();
    
    bool GGA = false;
    bool GSA = false;

    bool has = false;
    float last_alt = 0.0;
    float last_err = 0.0;

    while (true) {
        switch (data->gps->sample()) {
            case 0: {
                //CONSOLE("%s", gps.msg);
                continue;
            }

            case 1: {
                GGA = true;
                break;
            }

            case 2: {
                GSA = true;
                break;
            }
        }

        if (GGA && GSA) {
            double dt = t.elapsed_time().count() / 1000000.0;
            t.reset();

            GGA = false;
            GSA = false;


            float error = data->gps->vdop * 3.0 * data->GPS_WEIGHT;

            if (has)
                data->vel.update((data->gps->alt - last_alt) / dt, last_err + error);
            last_err = error;
            last_alt = data->gps->alt;
            has = true;

            data->alt.update(data->gps->alt, error);
        }

        
        


    }
}

void KalmanFilter::start_async(IMU* imu, Baro* baro, SerialGPS* gps) {
    this->async_data.running = true;
    this->async_data.imu = imu;
    this->async_data.baro = baro;
    this->async_data.gps = gps;

    thread_imu.start(callback(do_accel_async, &this->async_data));
    thread_baro.start(callback(do_baro_async, &this->async_data));
    //thread_gps.start(callback(do_gps_async, &this->async_data));
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

gpsData KalmanFilter::last_gps() {
    gpsData res = {
        this->async_data.gps->latitude,
        this->async_data.gps->longitude,
        this->async_data.gps->alt,
        this->async_data.gps->pdop,
        this->async_data.gps->hdop,
        this->async_data.gps->vdop,
        this->async_data.gps->time,
    };

    return res;
}