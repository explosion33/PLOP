#![allow(dead_code)]

extern crate bmp085;
extern crate i2cdev;

use bmp085::*;
use i2cdev::linux::*;
use i2cdev::sensors::{Barometer, Thermometer};

use std::time::{Duration, Instant};
use std::fs::File;

use std::io::{self, Read};
use std::io::Write;

use std::sync::{Arc, Mutex, MutexGuard};
use std::thread;


pub struct Baro {
    sensor: BMP085BarometerThermometer<LinuxI2CDevice>,
    config_path: String,
    pub base_alt: f32,
    pub base_pres: f32,

}

// TODO make baro code asyncronous

impl Baro {
    pub fn new(config_path: &str) -> Option<Baro> {
        let i2c_dev = match LinuxI2CDevice::new("/dev/i2c-1", BMP085_I2C_ADDR) {
            Ok(n) => n,
            Err(n) => {
                println!("{:?}", n);
                return None;
            },
        };

        let sensor = match BMP085BarometerThermometer::new(i2c_dev, SamplingMode::UltraHighRes) {
            Ok(n) => n,
            Err(n) => {
                println!("{:?}", n);
                return None;
            },
        };
    
        let (base_alt, base_pres): (f32, f32) = match Baro::read_config(String::from(config_path)) {
            Ok(n) => {n},
            Err(_) => {
                Baro::empty_configure(String::from(config_path));
                let res = match Baro::read_config(String::from(config_path)) {
                    Ok(n) => n,
                    Err(_) => {
                        println!("FATAL ERROR | could not find or create valid baro.conf");
                        return None;
                    },
                };
                println!("WARNING | Created blank config, readings will be wrong");
                res
            },
        };

        Some(Baro {
            sensor,
            config_path: String::from(config_path),
            base_alt,
            base_pres,
        })
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

    pub fn get_temp(&mut self) -> Option<f32> {
        match self.sensor.temperature_celsius() {
            Ok(n) => Some(n),
            Err(_) => None,
        }
    }

    pub fn get_pressure(&mut self) -> Option<f32> {
        match self.sensor.pressure_kpa() {
            Ok(n) => Some(n),
            Err(_) => None,
        }
    }

    // configures base altitude for a given known altitude
    // takes t >= 5 seconds
    // returns true if the config was successfull false otherwise
    pub fn configure(&mut self, alt: f32, iter: usize) -> bool {
        let mut file = match File::create(self.config_path.to_string()) {
            Ok(n) => n,
            Err(_) => {
                println!("Could not create file");
                return false;
            },
        };

        // calculate average pressure
        let mut total_pres: f32 = 0f32;
        let mut num_pres:usize = 0;
        while num_pres < iter {
            match self.get_pressure() {
                Some(n) => {
                    num_pres += 1;
                    total_pres += n;
                },
                None => {},
            };
            thread::sleep(Duration::from_millis(10));
        };
    
        let avg_pres = total_pres / num_pres as f32;

        // write data to config
        match file.write_all(format!("{}\n{}", alt, avg_pres).as_bytes()) {
            Ok(_) => {},
            Err(_) => {
                println!("could not write to file");
                return false;
            },
        };

        let _ = io::stdout().flush();

        self.base_alt = alt;
        self.base_pres = avg_pres;

        true
    }

    // creates an empty configuration file assuming
    // a pressure of 0Pa at 0m
    // returns true if successfull false otherwise 
    fn empty_configure(config_path: String) -> bool {
        let mut file = match File::create(config_path.to_string()) {
            Ok(n) => n,
            Err(_) => { 
                println!("could not creaet file");
                return false;
            }
        };

        match file.write_all(format!("{}\n{}", 0, 0).as_bytes()) {
            Ok(_) => {},
            Err(_) => {
                println!("could not write to file");
                return false;
            },
        }
        let _ = io::stdout().flush();
        
        false
    }

    // read stored configuraion values
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
            return Err("Failed to parse file, not enough values found");
        }

        let base_alt: f32 = match parts[0].parse::<f32>() {
            Ok(n) => {n},
            Err(_) => {return Err("Failed to parse file, invalid altitude format")}
        };
        let base_pres: f32 = match parts[1].parse::<f32>() {
            Ok(n) => {n},
            Err(_) => {return Err("Failed to parse file, invalid pressuer format")}
        };

        Ok((base_alt, base_pres))


    }

    // calculates the alitude based on the
    // current pressure and temperature
    // and the base pressure
    pub fn get_alt(&mut self) -> Option<f32> {
        let p = match self.get_pressure() {
            Some(n) => n,
            None => {return None;}
        };

        let t = match self.get_temp() {
            Some(n) => n + 273.15,
            None => {return None;}
        };
        
        //C to Kelvin
        const RATIO: f32 = 0.19022256;
        const RATIO2: f32 = 0.0065;

        let res: f32 = self.base_alt + ((((self.base_pres/p).powf(RATIO))-1.0)*t)/RATIO2;
        Some(res)
    }


    pub fn get_noise(&mut self, iter: usize) -> f32 {
        let mut vals: Vec<f32> = Vec::with_capacity(iter); 
        let mut mean: f32 = 0f32;
        let mut i: usize = iter;

        while i > 0 {
            match self.get_alt() {
                Some(alt) => {
                    vals.push(alt);
                    mean += alt;
                    i -= 1;
                },
                _ => {
                },
            };
        }
        
        mean /= iter as f32;

        let mut stdev: f32 = 0f32;

        for val in vals {
            stdev += (mean - val) * (mean - val);
        }

        stdev /= iter as f32;
        return stdev.sqrt();
    }
}
