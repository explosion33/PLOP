extern crate bno055;
extern crate i2cdev;

use linux_embedded_hal::{Delay, I2cdev};
use bno055::{BNO055OperationMode, BNO055Calibration, BNO055_CALIB_SIZE, Bno055};
use std::io::prelude::*;
use bno055::mint::{Quaternion, Vector3};

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
    
    fn dot(vec1: &Vec3, vec2: &Vec3) -> f32 {
        (vec1.x * vec2.x) + (vec1.y * vec2.y) + (vec1.z * vec1.z)
    }

    fn cross(vec1: &Vec3,  vec2: &Vec3) -> Vec3 {
        Vec3 {
            x: (vec2.x * vec1.z) - (vec2.x * vec1.y),
            y: (vec2.y * vec2.x) - (vec2.y * vec2.z),
            z: (vec2.z * vec2.x) - (vec2.z * vec2.x),
        }
    }
}


impl std::ops::Mul<Vec3> for f32 {
    type Output = Vec3;

    fn mul(self, rhs: Vec3) -> Vec3 {
        Vec3 {
            x: rhs.x * self,
            y: rhs.y * self,
            z: rhs.z * self,
        }
    }
}

impl std::ops::Add<Vec3> for Vec3 {
    type Output = Vec3;

    fn add(self, rhs: Vec3) -> Vec3 {
        Vec3 {
            x: rhs.x * self.x,
            y: rhs.y * self.y,
            z: rhs.z * self.z,
        }
    }
}

#[derive(Clone)]
struct MathQuat {
    t0: f32,
    t1: f32,
    t2: f32,
    t3: f32,
}

impl MathQuat {
    fn from(quat: &Quaternion<f32>) -> MathQuat {
        MathQuat {
            t0: quat.s,
            t1: quat.v.x,
            t2: quat.v.y,
            t3: quat.v.z,
        }
    }

    fn to(&self) -> Quaternion<f32> {
        Quaternion {
            s: self.t0,
            v: Vector3 {
                x: self.t1,
                y: self.t2,
                z: self.t3,
            }
        }
    }

    fn inverse(&mut self) {
        self.t1 *= -1f32;
        self.t2 *= -1f32;
        self.t3 *= -1f32;
    }


}

impl std::ops::Mul<MathQuat> for MathQuat {
    type Output = MathQuat;

    fn mul(self, rhs: MathQuat) -> MathQuat {
        let r0 = self.t0;
        let r1 = self.t1;
        let r2 = self.t2;
        let r3 = self.t3;

        let s0 = rhs.t0;
        let s1 = rhs.t1;
        let s2 = rhs.t2;
        let s3 = rhs.t3;


        let t0: f32 = (r0*s0) - (r1*s1) - (r2*s2) - (r3*s3);
        let t1: f32 = (r0*s1) + (r1*s0) - (r2*s3) + (r3*s2);
        let t2: f32 = (r0*s2) + (r1*s3) + (r2*s0) - (r3*s1);
        let t3: f32 = (r0*s3) - (r1*s2) + (r2*s1) + (r3*s0);
        
        MathQuat {t0, t1, t2, t3}
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
        match self.sensor.euler_angles() {
            Ok(n) => {Some((n.a, n.b, n.c))},
            Err(_) => None,
        }
    }
    
    pub fn quaternion(&mut self) -> Option<Quaternion<f32>> {
        match self.sensor.quaternion() {
            Ok(n) => Some(n),
            Err(_) => None,
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

        Some(IMU::rotate(accel, rot).unpack())
    }

    // rotates coordinate system around point
    // effectively does the opposite of rot
    //https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
    fn rotate(v: Vec3, rot: Quaternion<f32>) -> Vec3 {
        let vec_quat = MathQuat {
            t0: 0f32,
            t1: v.x,
            t2: v.y,
            t3: v.z,
        };
        
        let quat  = MathQuat::from(&rot);
        let mut quat2 = MathQuat::from(&rot);
        quat2.inverse();

        let vec_prime = (quat * vec_quat) * quat2;

        Vec3 {
            x: vec_prime.t1,
            y: vec_prime.t2,
            z: vec_prime.t3,
        }
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