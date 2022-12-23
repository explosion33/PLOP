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
        * better characterize noise of each sensor, perhaps build in weight system
        * extract Z component of GPS velocity (see TODO below)
        * ensure IMU acceleration rotations are working properly (not sure how to do this)
        * clean-up async code from barometer class (no longer using)
        * look into running multiple i2c lines on raspberry pi to seperate sensors
        * look into creating a seperate Kalman Filter to manage velocity
        *   update velocity based on predicted acceleromter
        *   calculated GPS
        *   and barometer estimated velocity
        */
        

    let mut imu = IMU::new("imu.conf");
    let mut baro = Baro::new("baro.conf").unwrap();
    let mut gps = GPS::new();

    baro.configure(189.7f32);
    //baro.start_async(10);

    
    //imu.calibrate();
    imu.calibrate_static_erorr();

    let mut start = Instant::now();

    let mut vel: f32 = 0f32;

    let mut filter = Filter {
        alt: 191f32,
        var: 0f32,
    };

    let mut accels: [f32; 10] = [0f32; 10];
    let mut baros: [f32; 10] = [0f32; 10];
    
    let mut iter: usize = 0;

    loop {
        let dt = start.elapsed().as_secs_f32();
        start = Instant::now();

        match imu.accel() {
            Some((_, _, mut z)) => {
                //filter.predict(vel*dt + 0.5*z*dt*dt, 1f32);
                //vel += z*dt;
                for i in 1..10 {
                    accels[i] = accels[i-1];
                }

                if z < 0.05f32 && z > -0.05f32 {
                    z = 0f32;
                }

                accels[0] = z;
            },
            _ => {},
        }

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

        match gps.get_data() {
            Some(data) => {
                match data.alt {
                    Some((alt, var)) => {
                        filter.update(alt, var);
                    },
                    _ => {},
                };

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

        if iter < 10 {
            iter += 1;
            continue;
        }

        let mut total = 0f32;
        for i in 0..10 {
            total += accels[i]
        }
        total /= 10f32;

        filter.predict(vel*dt*10f32 + 0.5*total*(10f32*dt*dt), 1f32);
        vel += total*dt*10f32;

        println!("acc: {}", total);

        total = 0f32;
        for i in 0..10 {
            total += baros[i]
        }
        total /= 10f32;

        filter.update(total, 0.5f32);
        println!("baro: {}", total);

        println!("alt: {:4.1}, var: {}, vel: {}, dt: {}", filter.alt, filter.var, vel, dt);
    }
    


    // FIRE IGNITER
}
