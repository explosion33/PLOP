// tuning constants
const ACCEL_ROLLING_AVERAGE: usize = 2;
const BARO_ROLLING_AVERAGE: usize = 10;

const ERROR_ITER_ACCEL: usize = 1000;
const ERROR_ITER_BARO:  usize = 200;

const ACCEL_WEIGHT: f32 = 1f32; // 1 is no modification (lower value is higher priority)
const BARO_WEIGHT: f32 = 2f32;
const BARO_VEL_WEIGHT: f32 = 4f32;

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
    pub val: f32,
    pub var: f32,
}

impl Filter {
    pub fn update(&mut self, mean: f32, var: f32) {
        self.val = (self.val*var + mean*self.var)/(self.var + var);
        self.var = 1f32/((1f32/self.var) + (1f32/var));
    }

    pub fn predict(&mut self, mean: f32, var: f32) {
        self.val += mean;
        self.var += var;
    }
}

fn main() {

    /*
        * TODO:
        * test effectiveness of performing a rolling average over accel and baro
        *   adjust window size and compare results
        * test GPS velocity corrections
        * ensure IMU acceleration rotations are working properly (not sure how to do this)
        * look into running multiple i2c lines on raspberry pi to seperate sensors
        * add baromter velocity to filter
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
    println!("{:?}", imu.get_calibration());
    
    imu.calibrate_static_erorr();


    println!("characterizing accel noise");
    let ACCEL_NOISE = imu.get_noise(ERROR_ITER_ACCEL);
    
    println!("characterizing baro noise");
    let BARO_NOISE = baro.get_noise(ERROR_ITER_BARO);

    println!("accel noise: {}, baro noise: {}", ACCEL_NOISE, BARO_NOISE);

    // ====================================================================
    // FILTER INITIALIZATION
    // ====================================================================

    // dt
    let mut start = Instant::now();

    let mut vel_filter = Filter {
        val: 0f32,
        var: 0f32,
    };

    let mut pos_filter = Filter {
        val: 0f32,
        var: 1000f32,
    }; // Kalman valitude

    // gps velocity storage
    let mut last_gps: [f32; 3] = [0f32; 3];
    let mut last_reading = Instant::now();
    let mut complete: bool = false;

    // baro velocity storage
    let mut last_baro_alt: f32 = 0f32;

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
                for i in 1..ACCEL_ROLLING_AVERAGE {
                    accels[i] = accels[i-1];
                }

                accels[0] = z;
            },
            _ => {},
        }

        // update baro rolling average
        match baro.get_alt() {
            Some(alt) => {
                for i in 1..BARO_ROLLING_AVERAGE {
                    baros[i] = baros[i-1];
                }
                baros[0] = alt;
            },
            _ => {
            },
        };


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


        pos_filter.predict(vel_filter.val*dt + 0.5*acc*(dt*dt), (ACCEL_NOISE * ACCEL_WEIGHT * dt) + vel_filter.var);
        vel_filter.predict(acc*dt, ACCEL_NOISE*ACCEL_WEIGHT*dt);

        //println!("baro_vel: {}, var: {}", (baro_alt - last_baro_alt) * dt, BARO_NOISE * BARO_NOISE * BARO_VEL_WEIGHT);

        // update from baro
        pos_filter.update(baro_alt, BARO_NOISE * BARO_WEIGHT);
        vel_filter.update((baro_alt - last_baro_alt) * dt, BARO_NOISE * BARO_NOISE * BARO_VEL_WEIGHT);
        last_baro_alt = baro_alt;

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

                let mut get_heading = |data: &GpsData| -> Option<(f32, f32, f32)> {
                    match data.lat {
                        Some((lat, _)) => {
                            match data.long {
                                Some((long, _)) => {
                                    match data.alt {
                                        Some((alt, _)) => {
                                            let mut res = None;
                                            if complete {
                                                println!("lat: {lat}, long: {long} last: {:?}", last_gps);
                                                let x: f32 = (lat - last_gps[0]) * 111119f32;
                                                let y: f32 = (long - last_gps[1]) * 111119f32;
                                                let z: f32 = alt - last_gps[2];
                                                res = Some((x,y,z));
                                            }

                                            last_gps[0] = lat;
                                            last_gps[1] = long;
                                            last_gps[2] = alt;
                                            complete = true;
                                            res
                                        },
                                        None => None,
                                    }
                                },
                                None => None,
                            }
                        },
                        None => None,
                    }
                };

                // update tracked speed
                match data.speed {
                    Some((speed, var)) => {

                        match get_heading(&data) {
                            Some((x,y,z)) => {
                                let mut speed_z = 0f32;
                                if (x + y + z) != 0f32 {
                                    let speed_z = speed*(z/(x+y+z));
                                }
                                vel_filter.update(speed_z, var*3f32);
                                println!("=========({}, {}, {}) speed: {} adjusted: {}=========", x, y, z, speed, speed_z);
                            },
                            _ => {},
                        }
                    },
                    _ => {},
                }

            },
            _ => {},
        }

        //println!("baro: {}", baro_alt);
        //println!("acc: {}", acc);
        println!("alt: {:4.1}, var: {}, vel: {}, var: {}, dt: {}", pos_filter.val, pos_filter.var, vel_filter.val, vel_filter.var, dt);
    }
    


    // FIRE IGNITER
}