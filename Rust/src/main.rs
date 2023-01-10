// tuning constants
const BARO_ROLLING_AVERAGE: usize = 100;

const ERROR_ITER_ACCEL: usize = 1000;
const ERROR_ITER_BARO:  usize = 200;

const ACCEL_WEIGHT: f32 = 1f32; // 1 is no modification (lower value is higher priority)
const BARO_WEIGHT: f32 = 60f32;
const BARO_VEL_WEIGHT: f32 = 200f32;

// replace later
const INITIAL_ALT: f32 = 48f32;
const I2C_FREQ_RATIO: usize = 11; // (IMU hz) / (BARO hz)

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

struct KalmanFilter {
    start: Instant,
    pub velocity: Filter,
    pub altitude: Filter,

    baro_iter: usize,

    // gps velocity storage
    last_gps: [f32; 3],
    complete: bool,

    // baro velocity storage
    last_baro_alt: f32,

    // rolling averages
    filtered_accel: f32,
    baros: [f32; BARO_ROLLING_AVERAGE],

    ACCEL_WEIGHT: f32,
    BARO_WEIGHT: f32,
    BARO_VEL_WEIGHT: f32,
    GPS_WEIGHT: f32,

}

impl KalmanFilter {
    pub fn new(alt: f32, accel_weight: f32, baro_weight: f32, baro_vel_weight: f32, gps_weight: f32) -> KalmanFilter {
        KalmanFilter {
            start: Instant::now(),
            velocity: Filter {
                val: 0f32,
                var: 0f32,
            },
            altitude: Filter {
                val: alt,
                var: 0f32,
            },
            baro_iter: I2C_FREQ_RATIO,
            last_gps: [0f32; 3],
            complete: false,
            last_baro_alt: alt,
            filtered_accel: 0f32,
            baros: [0f32; BARO_ROLLING_AVERAGE],

            ACCEL_WEIGHT: accel_weight,
            BARO_WEIGHT: baro_weight,
            BARO_VEL_WEIGHT: baro_vel_weight,
            GPS_WEIGHT: gps_weight,
        }
    }

    fn tick_accel(&mut self, imu: &mut IMU) -> f32 {
        match imu.accel() {
            Some((_, _, z)) => {
                //self.filtered_accel = ((1.0 - ACCEL_LOW_PASS) * self.filtered_accel) + (ACCEL_LOW_PASS);
                z
            }
            None => {
                0f32
            },
        }

        //self.filtered_accel
        
    }

    fn tick_baro(&mut self, baro: &mut Baro) -> f32 {
        match baro.get_alt() {
            Some(alt) => {
                for i in 1..BARO_ROLLING_AVERAGE {
                    self.baros[i] = self.baros[i-1];
                }
                self.baros[0] = alt;
            },
            _ => {
            },
        };

        let mut baro_alt = 0f32;
        for i in 0..BARO_ROLLING_AVERAGE {
            baro_alt += self.baros[i]
        }
        baro_alt /= BARO_ROLLING_AVERAGE as f32;
        return baro_alt;
    }

    pub fn tick(&mut self, imu: &mut IMU, baro: &mut Baro, gps: &mut GPS) {
        let dt = self.start.elapsed().as_secs_f32();
        self.start = Instant::now();


        let acc = self.tick_accel(imu);

        self.altitude.predict(self.velocity.val*dt + 0.5*acc*(dt*dt), (self.ACCEL_WEIGHT * dt) + self.velocity.var*dt);
        self.velocity.predict(acc*dt, self.ACCEL_WEIGHT*dt);

        if self.baro_iter == 0 {
            self.baro_iter = I2C_FREQ_RATIO;

            let baro_alt = self.tick_baro(baro);
            println!("baro_alt: {}", baro_alt);

            // update from baro
            self.altitude.update(baro_alt, self.BARO_WEIGHT*dt);
            self.velocity.update((baro_alt - self.last_baro_alt) * dt, self.BARO_VEL_WEIGHT*dt);
            self.last_baro_alt = baro_alt;
        }
        self.baro_iter -= 1;


        

        // update from GPS
        match gps.get_data() {
            Some(data) => {
                println!("{:?}", data);
                // update altitude
                match data.alt {
                    Some((alt, var)) => {
                        self.altitude.update(alt, var*self.GPS_WEIGHT);
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
                                            if self.complete {
                                                //println!("lat: {lat}, long: {long} last: {:?}", last_gps);
                                                let x: f32 = (lat - self.last_gps[0]) * 111119f32;
                                                let y: f32 = (long - self.last_gps[1]) * 111119f32;
                                                let z: f32 = alt - self.last_gps[2];
                                                res = Some((x,y,z));
                                            }

                                            self.last_gps[0] = lat;
                                            self.last_gps[1] = long;
                                            self.last_gps[2] = alt;
                                            self.complete = true;
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
                                self.velocity.update(speed_z, self.GPS_WEIGHT*var*3f32);
                            },
                            _ => {},
                        }
                    },
                    _ => {},
                };
            },
            _ => {},
        }
    }
}

fn main() {

    /*
        * TODO:
        * test effectiveness of performing a rolling average over accel and baro
        *   adjust window size and compare results
        * test GPS velocity corrections
        * ensure IMU acceleration rotations are working properly
        *   brief testing it looks like one of the axis is inverted with respect to Z
        * look into running multiple i2c lines on raspberry pi to seperate sensors
        */
        

    // ====================================================================
    // INIT AND CALIBRATE SENSORS
    // ====================================================================
    println!("getting sensors");
    let mut imu = IMU::new("imu.conf");
    let mut baro = Baro::new("baro.conf").unwrap();
    let mut gps = GPS::new();

    //println!("{:?}", imu.get_frequency());
    //println!("{:?}", baro.get_frequency());

    // TODO: configure baro based on GPS altitude?
    println!("Calibrating Barometer");
    baro.configure(INITIAL_ALT, 100);
    println!("done\n");
    
    println!("calibrating IMU");
    //imu.calibrate();
    println!("{:?}\n", imu.get_calibration());
    
    println!("getting acceleromter offests");
    let (res_x, res_y, res_z) = imu.calibrate_static_erorr();
    println!("x: {}, y: {}, z: {}\n", res_x, res_y, res_z);

    println!("characterizing accel noise");
    let ACCEL_NOISE = imu.get_noise(ERROR_ITER_ACCEL);
    
    println!("characterizing baro noise");
    let BARO_NOISE = baro.get_noise(ERROR_ITER_BARO);

    println!("accel noise: {}, baro noise: {}", ACCEL_NOISE, BARO_NOISE);

    

    let mut filter = KalmanFilter::new(
        INITIAL_ALT,
        ACCEL_NOISE*ACCEL_WEIGHT,
        BARO_NOISE*BARO_WEIGHT,
        BARO_VEL_WEIGHT*(BARO_NOISE*BARO_NOISE),
        1f32
    );

    let mut i = 0;
    let mut vals: Vec<f32> = Vec::with_capacity(1000);
    let mut z_accels: Vec<f32> = Vec::with_capacity(1000);
    let mut baro_alts: Vec<f32> = Vec::with_capacity(1000);
    loop {
        filter.tick(&mut imu, &mut baro, &mut gps);
        

        println!("alt: {:4.1}, var: {}, vel: {}, var: {}, z: {}, {}", filter.altitude.val, filter.altitude.var, filter.velocity.val, filter.velocity.var, imu.last_accel().unwrap().2, i);
        
        /*if i >= 1000 {
            vals.push(filter.altitude.val);
            z_accels.push(imu.last_accel().unwrap().2);
            baro_alts.push(baro.last_alt().unwrap());
        }

        if i >= 2000 {
            println!("F={:?}\nA={:?}\nB={:?}", vals, z_accels, baro_alts);
            return;
        }

        i += 1;*/
    }
}