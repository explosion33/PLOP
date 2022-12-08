//use crate::igniter::Igniter;
//mod igniter;

use crate::baro::Baro;
mod baro;

use crate::imu::IMU;
mod imu;

use crate::gps::{GPS, GpsData};
mod gps;

//use crate::indicator::Indicator;
//mod indicator;

fn main() {

    let mut imu = IMU::new("imu.conf");
    let mut baro = Baro::new("bar.conf").unwrap();
    let mut gps = GPS::new();
    //imu.calibrate();

    loop {
        match gps.get_data() {
            Some(data) => {
                println!("\n{:?}", data)
            },
            _ => {
                //print!(".");
                //stdout().flush();
                //thread::sleep(Duration::from_millis(50));
            },
        }

        let (a,b,c) = match imu.euler() {
            Some(n) => n,
            _ => {
                println!("lost value");
                (0f32,0f32,0f32)
            },
        };

        let (t0, t1, t2, t3) = match imu.quaternion() {
            Some(n) => {
                (n.s, n.v.x, n.v.y, n.v.z)
            },
            _ => {
                println!("lost value");
                (0f32, 0f32, 0f32, 0f32)
            }
        };
        
        match imu.accel() {
            Some((x,y,z)) => {
                println!("{:8.0}, {:8.0}, {:8.0} | {:8.0}, {:8.0}, {:8.0} | {:8.4}, {:8.4}, {:8.4}, {:8.4}", x.abs(), y.abs(), z.abs(), a, b, c, t0, t1, t2, t3);
            },
            _ => {},
        }
        
        match baro.get_alt_variance(10, None) {
            Ok((alt, var)) => {
                println!("baro: {alt}, {var}");
            },
            Err(_) => todo!(),
        }
    }
    


    // FIRE IGNITER
}
