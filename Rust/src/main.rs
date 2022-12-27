// tuning constants
const ACCEL_ROLLING_AVERAGE: usize = 10;
const BARO_ROLLING_AVERAGE: usize = 10;

const ERROR_ITER_ACCEL: usize = 1000;
const ERROR_ITER_BARO:  usize = 200;

const ACCEL_WEIGHT: f32 = 1f32; // 1 is no modification (lower value is higher priority)
const BARO_WEIGHT: f32 = 1f32;

//use crate::igniter::Igniter;
//mod igniter;

use crate::baro::Baro;
mod baro;

use crate::imu::IMU;
mod imu;

use crate::gps::{GPS, GpsData};
mod gps;

use std::io::Write;
use std::io::stdout;
use std::thread;
use std::time::{Duration, Instant};

//use crate::indicator::Indicator;
//mod indicator;

struct Filter {
    pub alt: f32,
    pub  var: f32,
}

impl Filter {
    pub fn update(&mut self, mean: f32, var: f32) {
        self.alt = (self.alt*var + mean*self.var)/(self.var + var);
        self.var = 1f32/((1f32/self.var) + (1f32/var));
    }

    pub fn predict(&mut self, mean: f32, var: f32) {
        self.alt += mean;
        self.var += mean;
    }
}

fn main() {

    /*
        * TODO:
        * test effectiveness of performing a rolling average over accel and baro
        *   adjust window size and compare results
        * extract Z component of GPS velocity (see TODO below)
        * ensure IMU acceleration rotations are working properly (not sure how to do this)
        * look into running multiple i2c lines on raspberry pi to seperate sensors
        * look into creating a seperate Kalman Filter to manage velocity
        *   update velocity based on predicted acceleromter
        *   calculated GPS
        *   and barometer estimated velocity
        */
        

    // ====================================================================
    // INIT AND CALIBRATE SENSORS
    // ====================================================================
    let mut imu = IMU::new("imu.conf");
    let mut baro = Baro::new("baro.conf").unwrap();
    let mut gps = GPS::new();

    // TODO: configure baro based on GPS altitude?
    baro.configure(189.7f32, 50);

    
    //imu.calibrate();
    imu.calibrate_static_erorr();

    // ====================================================================
    // CHARACTERIZE SENSOR NOISE
    // ====================================================================
    println!("characterizing accel noise");
    let mut i: usize = ERROR_ITER_ACCEL;
    let mut vals: [f32; ERROR_ITER_ACCEL] = [0f32; ERROR_ITER_ACCEL];
    let mut mean: f32 = 0f32;
    while i > 0 {
        match imu.accel() {
            Some((_, _, z)) => {
                vals[i-1] = z;
                mean += z;
                i -= 1;
            },
            _ => {},
        };
    }
    mean /= ERROR_ITER_ACCEL as f32;

    let mut stdev: f32 = 0f32;

    for i in 0..ERROR_ITER_ACCEL {
        stdev += (mean - vals[i]) * (mean - vals[i]);
    }

    stdev /= ERROR_ITER_ACCEL as f32;
    let ACCEL_NOISE: f32 = stdev.sqrt();

    

    println!("characterizing baro noise");
    i = ERROR_ITER_BARO;
    let mut vals: [f32; ERROR_ITER_BARO] = [0f32; ERROR_ITER_BARO];
    let mut mean: f32 = 0f32;

    while i > 0 {
        match baro.get_alt() {
            Some(alt) => {
                vals[i-1] = alt;
                mean += alt;
                i -= 1;
            },
            _ => {
            },
        };
    }
    
    mean /= ERROR_ITER_BARO as f32;

    let mut stdev: f32 = 0f32;

    for i in 0..ERROR_ITER_BARO {
        stdev += (mean - vals[i]) * (mean - vals[i]);
    }

    stdev /= ERROR_ITER_BARO as f32;
    let BARO_NOISE: f32 = stdev.sqrt();

    println!("accel noise: {}, baro noise: {}", ACCEL_NOISE, BARO_NOISE);

    // ====================================================================
    // FILTER INITIALIZATION
    // ====================================================================

    // dt
    let mut start = Instant::now();

    // tracked values
    let mut vel: f32 = 0f32;

    let mut pos_filter = Filter {
        alt: 191f32,
        var: 0f32,
    }; // Kalman altitude

    // rolling averages
    let mut accels: [f32; ACCEL_ROLLING_AVERAGE] = [0f32; ACCEL_ROLLING_AVERAGE];
    let mut baros: [f32; BARO_ROLLING_AVERAGE] = [0f32; BARO_ROLLING_AVERAGE];
    
    // max iters before all rolling averages are filled
    let mut iter: usize = ACCEL_ROLLING_AVERAGE.max(BARO_ROLLING_AVERAGE);


    // ====================================================================
    // MAIN KALMAN LOOP
    // ====================================================================

    loop {
        let dt = start.elapsed().as_secs_f32();
        start = Instant::now();

        // ====================================================================
        // READINGS AND AVERAGES
        // ====================================================================

        // update accel rolling average
        match imu.accel() {
            Some((_, _, mut z)) => {
                //pos_filter.predict(vel*dt + 0.5*z*dt*dt, 1f32);
                //vel += z*dt;
                for i in 1..10 {
                    accels[i] = accels[i-1];
                }

                // TODO tweak values perhaps make these come from stdev?
                // filter out values that are likely noise
                if z < 0.05f32 && z > -0.05f32 {
                    z = 0f32;
                }

                accels[0] = z;
            },
            _ => {},
        }

        // update baro rolling average
        match baro.get_alt() {
            Some(alt) => {
                //filter.update(alt, 0.5f32);
                //println!("{}", alt);

                for i in 1..10 {
                    baros[i] = baros[i-1];
                }
                baros[0] = alt;
            },
            _ => {
            },
        };

        // fill imu and baro windows before averaging
        if iter > 0 {
            iter -= 1;
            continue;
        }

        // average accel readings
        let mut acc = 0f32;
        for i in 0..ACCEL_ROLLING_AVERAGE {
            acc += accels[i]
        }
        acc /= ACCEL_ROLLING_AVERAGE as f32;

        // average barometer readings
        let mut baro_alt = 0f32;
        for i in 0..BARO_ROLLING_AVERAGE {
            baro_alt += baros[i]
        }
        baro_alt /= BARO_ROLLING_AVERAGE as f32;


        // ====================================================================
        // KALMAN UPDATES
        // ====================================================================

        // TODO track and combine velocity error
        // dx = V_i * t + 1/2at^2
        pos_filter.predict(vel*dt + 0.5*acc*(dt*dt), ACCEL_NOISE * ACCEL_WEIGHT);
        vel += acc*dt*10f32;

        // update from baro
        pos_filter.update(baro_alt, BARO_NOISE * BARO_WEIGHT);

        // update from GPS
        match gps.get_data() {
            Some(data) => {
                // update altitude
                match data.alt {
                    Some((alt, var)) => {
                        pos_filter.update(alt, var);
                    },
                    _ => {},
                };

                // update tracked speed
                match data.speed {
                    Some((speed, _)) => {
                        println!("gps_vel: {}", speed);
                        // TODO: use orientation to get Z componenet of velocity?
                        // may not be possible without the heading componenent?
                        // alternitively estimate velocity in all directions using accelerometer
                        // use these 3 to compute overall heading
                        vel = speed;
                    },
                    _ => {},
                }
            },
            _ => {},
        }

        println!("baro: {}", baro_alt);
        println!("acc: {}", acc);
        println!("alt: {:4.1}, var: {}, vel: {}, dt: {}", pos_filter.alt, pos_filter.var, vel, dt);
    }
    


    // FIRE IGNITER
}