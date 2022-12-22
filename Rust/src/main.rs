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
        * better characterize noise of each sensor, perhaps build in weight system
        * add GPS altitude measurement with filter.update
        * update vel using the gps' velocity calculation (may require rotating the velocity by quaternion)
        * add more in-depth calibration to ssimu
        *   make sure device is fully calibrated (internal blackbox)
        *   add external acceleration offsets based on average resting accelerations
        * ensure IMU acceleration rotations are working properly (not sure how to do this)
        * clean-up async code from barometer class (no longer using)
        * look into running multiple i2c lines on raspberry pi to seperate sensors
        */
        

    let mut imu = IMU::new("imu.conf");
    let mut baro = Baro::new("baro.conf").unwrap();

    //baro.configure(191f32);
    //baro.start_async(10);

    //let mut gps = GPS::new();
    //imu.calibrate();

    let mut start = Instant::now();

    let mut vel: f32 = 0f32;

    let mut filter = Filter {
        alt: 191f32,
        var: 0f32,
    };

    loop {
        let dt = start.elapsed().as_secs_f32();
        start = Instant::now();

        match imu.accel() {
            Some((_, _, z)) => {
                filter.predict(vel*dt + 0.5*z*dt*dt, 1f32);
                vel += z*dt;
            },
            _ => {},
        }

        match baro.get_alt() {
            Some(alt) => {
                filter.update(alt, 0.5f32);
                println!("{}", alt);
            },
            _ => {
            },
        };

        println!("alt: {:4.1}, var: {}, vel: {}, dt: {}", filter.alt, filter.var, vel, dt);
    }
}
