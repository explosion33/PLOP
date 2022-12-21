extern crate bno055;
extern crate i2cdev;
extern crate quaternion_core;

use linux_embedded_hal::{Delay, I2cdev};
use bno055::{BNO055OperationMode, BNO055Calibration, BNO055_CALIB_SIZE, Bno055};
use std::io::prelude::*;
//use bno055::mint::{Quaternion};


use quaternion_core::{Vector3, Quaternion, conj, frame_rotation, point_rotation, from_euler_angles, to_euler_angles, RotationType, RotationSequence};

use std::time::Instant;

use std::fs::OpenOptions;

#[derive(Clone)]
struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    fn new() -> Vec3 {
        return Vec3 {x: 0f32, y: 0f32, z: 0f32};
    }

    fn unpack(&self) -> (f32, f32, f32) {
        return (self.x, self.y, self.z);
    }
}


pub struct IMU {
    sensor: Bno055<I2cdev>,
    path: String,
    vel: Vec3,
    disp: Vec3,
    dr_last: Option<Instant>,
}

impl IMU {
    pub fn new(path: &str) -> IMU {
        let dev = I2cdev::new("/dev/i2c-1").unwrap();
        let mut delay = Delay {};
        let mut sensor = Bno055::new(dev).with_alternative_address();
        sensor.init(&mut delay).expect("An error occurred while building the IMU");
        sensor.set_mode(BNO055OperationMode::NDOF, &mut delay).unwrap();

        
        let mut file = match OpenOptions::new()
        .read(true)
        .open(path) {
            Ok(n) => n,
            Err(_) => {
                println!("Could not find configuration file");
                return IMU {
                    sensor,
                    path: path.to_string(),
                    vel: Vec3::new(),
                    disp: Vec3::new(),
                    dr_last: None,
                };
            }
        };

        let mut buf = [0u8; BNO055_CALIB_SIZE];

        file.read(&mut buf).unwrap();

        let calib = BNO055Calibration::from_buf(&buf);
        sensor.set_calibration_profile(calib, &mut delay).unwrap();


        IMU {
            sensor,
            path: path.to_string(),
            vel: Vec3::new(),
            disp: Vec3::new(),
            dr_last: None,
        }
    }

    pub fn euler(&mut self) -> Option<(f32, f32, f32)> {
        // TODO calculate from quaternion as recommended by bosch (Ndof firmware issue)
        /*match self.sensor.euler_angles() {
            Ok(n) => {Some((n.a, n.b, n.c))},
            Err(_) => None,
        }*/

        match self.quaternion() {
            Some(n) => {
                let v = to_euler_angles(RotationType::Extrinsic, RotationSequence::XYZ, n);
                const TODEG: f32 = 57.29577951308;
                Some((v[0]*TODEG, v[1]*TODEG, v[2]*TODEG))
            },
            None => None
        }
    }
    
    pub fn quaternion(&mut self) -> Option<Quaternion<f32>> {
        match self.sensor.quaternion() {
            Ok(quat) => {
                Some((quat.s, [quat.v.x, quat.v.y, quat.v.z])) //mint quaternion to math quaternion
            },
            Err(_) => None,
        }
    }

    // rotates coordinate system around point
    // effectively does the opposite of rot
    //https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
    fn rotate(v: Vec3, rot: Quaternion<f32>) -> Vec3 {
        //let quat = quaternion::conj(rot);
        //not sure why but ZYX appears to be the correct way to rotate our vectors?
        let res = frame_rotation(rot, [v.z, v.y, v.x]);

        Vec3 {
            x: res[2],
            y: res[1],
            z: res[0],
        }
    }

    pub fn accel(&mut self) -> Option<(f32, f32, f32)> {
        //TODO: rotate based on current orientation
        let accel = match self.sensor.linear_acceleration() {
            Ok(n) => {
                Vec3 {x: n.x, y: n.y, z: n.z}
            }
            Err(_) => {return None;},
        };

        let rot = match self.quaternion() {
            Some(n) => n,
            None => {
                return None;
            }
        };

        //Some(accel.unpack())
        Some(IMU::rotate(accel, rot).unpack())
    }

    pub fn dr_tick(&mut self) -> (f32, f32, f32) {
        if self.dr_last == None {
            self.dr_last = Some(Instant::now());
            return self.disp.unpack();
        }

        let dt: f32 = self.dr_last.unwrap().elapsed().as_micros() as f32 / 1000000f32;
        println!("{}", dt);

        self.disp.x += self.vel.x * dt;
        self.disp.y += self.vel.y * dt;
        self.disp.z += self.vel.z * dt;

        match self.accel() {
            Some((x,y,z)) => {
                self.vel.x += x * dt;
                self.vel.y += y * dt;
                self.vel.z += z * dt;
            },
            None => {},
        };

        self.dr_last = Some(Instant::now());

        self.disp.unpack()
    }

    pub fn dr_update_vel(&mut self, x: f32, y: f32, z: f32) {
        self.vel.x = x;
        self.vel.y = y;
        self.vel.z = z;
    }  

    pub fn dr_update_disp(&mut self, x: f32, y: f32, z: f32) {
        self.disp.x = x;
        self.disp.y = y;
        self.disp.z = z;
    }

    pub fn calibrate(&mut self) {
        let mut file = OpenOptions::new()
            .write(true)
            .create(true)
            .open(&self.path)
            .unwrap();



        while !self.sensor.is_fully_calibrated().unwrap() {
            print!("{:#?}", self.sensor.get_calibration_status().unwrap());
        }

        let calib = self.sensor.calibration_profile(&mut Delay {}).unwrap();

        file.write_all(calib.as_bytes());
    }

}

#[test]
fn test() {
    // rotated vector (device is 90 deg)
    let rot = Vec3 {
        x: 0f32,
        y: 0f32,
        z: 1f32,
    };

    const TOPI: f32 = 0.01745f32;

    let quat = from_euler_angles(RotationType::Extrinsic, RotationSequence::XYZ, [45f32 * TOPI, 45f32 * TOPI, 0f32 * TOPI]);

    let v = to_euler_angles(RotationType::Extrinsic, RotationSequence::XYZ, quat);

    println!("(yaw, pitch, roll)\n(a,g,b)\n({}, {}, {})\n{:?}", v[0]/TOPI, v[1]/TOPI, v[2]/TOPI, quat);

    let (x,y,z) = IMU::rotate(rot, quat).unpack();

    println!("{}, {}, {}", x, y, z);

    assert!(false);
}