#![allow(dead_code)]

extern crate bmp085;
extern crate i2cdev;

use bmp085::*;
use i2cdev::linux::*;
use i2cdev::sensors::{Barometer, Thermometer};

use std::thread;
use std::time::Duration;
use std::fs::File;

use std::io::{self, Read};
use std::io::Write;


pub struct Baro {
    sensor: Box<BMP085BarometerThermometer<LinuxI2CDevice>>,
    config_path: String,
    pub base_alt: f32,
    pub base_pres: f32,
}

impl Baro {
    pub fn new(config_path: &str) -> Baro {
        let i2c_dev = LinuxI2CDevice::new("/dev/i2c-1", BMP085_I2C_ADDR).expect("unable to access i2c device");
        let sensor = BMP085BarometerThermometer::new(i2c_dev, SamplingMode::Standard).expect("could not create bmp085 object");
    
        let (base_alt, base_pres): (f32, f32) = match Baro::read_config(String::from(config_path)) {
            Ok(n) => {n},
            Err(_) => {
                Baro::empty_configure(String::from(config_path));
                let res = Baro::read_config(String::from(config_path))
                    .expect("FATAL ERROR | could not find or create valid baro.conf");
                println!("WARNING | Created blank config, readings will be wrong");
                res
            },
        };

        Baro {sensor: Box::new(sensor), config_path: String::from(config_path), base_alt, base_pres}
    }

    pub fn has_configuration(&self) -> bool {
        let _ = match File::open(self.config_path.to_string()) {
            Ok(_) => {
                return true;
            },
            Err(_) => {
                return false;
            }
        };
    }

    pub fn get_temp(&mut self) -> Result<f32, u8> {
        match self.sensor.temperature_celsius() {
            Ok(n) => {Ok(n)},
            Err(_) => {Err(0)},
        }
    }

    pub fn get_pressure(&mut self) -> Result<f32, u8> {
        match self.sensor.pressure_kpa() {
            Ok(n) => {Ok(n)},
            Err(_) => {Err(0)},
        }
    }

    pub fn configure(&mut self, alt: f32) {
        let mut file = File::create(self.config_path.to_string()).expect("could not create file");

        let mut total_pres: f32 = 0f32;
        let mut num_pres:usize = 0;
        while num_pres < 50 {
            match self.get_pressure() {
                Ok(n) => {
                    num_pres += 1;
                    total_pres += n;
                },
                Err(_) => {},
            };
            thread::sleep(Duration::from_millis(100));

        }
    
        let avg_pres = total_pres / num_pres as i32 as f32;
        println!("{}, {}, {}", avg_pres, total_pres, num_pres);

        file.write_all(format!("{}\n{}", alt, avg_pres).as_bytes()).expect("could not write to file");
        let _ = io::stdout().flush();

        self.base_alt = alt;
        self.base_pres = avg_pres;
        
    
    }

    fn empty_configure(config_path: String) {
        let mut file = File::create(config_path.to_string()).expect("could not create file");

        file.write_all(format!("{}\n{}", 0, 0).as_bytes()).expect("could not write to file");
        let _ = io::stdout().flush();
    }

    fn read_config(path: String) -> Result<(f32, f32), &'static str> {
        let mut file = match File::open(path.to_string()) {
            Ok(n) => {n},
            Err(_) => {
                return Err("Could not open config file");
            }
        };

        let mut buf: Vec<u8> = vec![];

        let text: String = match file.read_to_end(&mut buf) {
            Ok(_) => {
                let s: String = match String::from_utf8(buf) {
                    Ok(n) => {n},
                    Err(_)=> {
                        return Err("could not read config, file contains invalid characeters")
                    }
                };
                s
            },
            Err(_) => {
                return Err("could not read config");
            }
        };

        let parts: Vec<&str> = text.trim().split("\n").collect();

        if parts.len() != 2 {
            return Err("Invalid config");
        }

        let base_alt: f32 = match parts[0].parse::<f32>() {
            Ok(n) => {n},
            Err(_) => {return Err("Invalid config")}
        };
        let base_pres: f32 = match parts[1].parse::<f32>() {
            Ok(n) => {n},
            Err(_) => {return Err("Invalid config")}
        };

        Ok((base_alt, base_pres))


    }

    pub fn get_alt(&mut self) -> Result<f32, u8> {
        let p = match self.get_pressure() {
            Ok(n) => {n},
            Err(_) => {return Err(0u8)}
        };
        let t =match self.get_temp() {
            Ok(n) => {n + 273.15},
            Err(_) => {return Err(0u8)}
        }; //C to Kelvin
        const RATIO: f32 = 0.19022256;
        const RATIO2: f32 = 0.0065;

        let res: f32 = self.base_alt + ((((self.base_pres/p).powf(RATIO))-1.0)*t)/RATIO2;
        Ok(res)
    }


}
