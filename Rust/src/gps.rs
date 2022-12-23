use std::env;

use adafruit_gps::{Gps, GpsSentence};
use adafruit_gps::gga::GgaData;
use adafruit_gps::NmeaOutput;
use adafruit_gps::set_baud_rate;

use std::sync::{Arc, Mutex};
use std::thread;

// TODO:
//  experiment with increasing speeds (current 1Hz, max 10Hz)
//  research the best way to get error
//      (current DOP * 3m for maximum potential error)

#[derive(Debug, Clone)]
struct Data {
    pub utc: f64, // seconds since last hour?
    pub lat: Option<f32>,
    pub long: Option<f32>,
    pub alt: Option<f32>, // above average sea level
    pub speed: Option<f32>, //vel m/s
    pub heading: Option<f32>, // degrees from mag_north
    pub hdop: Option<f32>,
    pub pdop: Option<f32>,
    pub vdop: Option<f32>,
    pub gga: bool,
    pub gsa: bool,
    pub vtg: bool,
}

impl Data {
    pub fn new() -> Data {
        Data {
            utc: 0f64,
            lat: Some(0f32),
            long: Some(0f32),
            alt: Some(0f32),
            speed: Some(0f32),
            heading: Some(0f32),
            hdop: Some(0f32),
            pdop: Some(0f32),
            vdop: Some(0f32),
            gga: false,
            gsa: false,
            vtg: false,
        }
    }

    pub fn ready(&self) -> bool {
        return self.gga && self.gsa && self.vtg
    }

    pub fn toUser(&self) -> GpsData {
        let lat = match self.lat {
            Some(n) => {
                let err = 3.0 * self.hdop.unwrap_or(60f32); // err is standard error times dilution
                Some((n, err))
            }
            None => None,
        };

        let long = match self.long {
            Some(n) => {
                let err = 3.0 * self.hdop.unwrap_or(60f32); // err is standard error times dilution
                Some((n, err))
            }
            None => None,
        };

        let speed = match self.speed {
            Some(n) => {
                let err = 0.1 * self.hdop.unwrap_or(60f32); // err from spec times dilution
                Some((n, err))
            }
            None => None,
        };

        let heading = self.heading;

        let alt = match self.alt {
            Some(n) => {
                let err = 3.0 * self.vdop.unwrap_or(60f32); // err is standard error times dilution
                Some((n, err))
            }
            None => None,
        };

        GpsData {
            lat,
            long,
            speed,
            heading,
            alt,
        }
    }
}

#[derive(Debug)]
pub struct GpsData {
    pub lat: Option<(f32, f32)>,
    pub long: Option<(f32, f32)>,
    pub speed: Option<(f32, f32)>,
    pub heading: Option<f32>,
    pub alt: Option<(f32, f32)>,
}

pub struct GPS {
    gps: Arc<Mutex<Gps>>,
    async_run: Arc<Mutex<bool>>,
    latest_data: Arc<Mutex<Data>>,
}

impl GPS {
    pub fn new() -> GPS {
        //println!("setting baud rate");
        //let r = set_baud_rate("57600", "/dev/serial0");
        //println!("{:?}", r);

        let mut gps = Gps::new("/dev/serial0", "9600");

        gps.pmtk_314_api_set_nmea_output(NmeaOutput{gga: 1, gsa: 1, gsv: 0,  gll: 0, rmc: 0, vtg: 1, pmtkchn_interval: 1 });
        let r = gps.pmtk_220_set_nmea_updaterate("100");
        println!("{:?}", r);

        let gps = Arc::new(Mutex::new(gps));
        let async_run = Arc::new(Mutex::new(true));
        let latest_data = Arc::new(Mutex::new(Data::new()));

        let gps_clone = Arc::clone(&gps);
        let async_run_clone = Arc::clone(&async_run);
        let latest_data_clone = Arc::clone(&latest_data);

        thread::spawn(move || {
            let mut gps = gps_clone.lock().expect("could not lock");

            loop {
                let values = gps.update();
                //println!("{:?}", values);
        
                // Depending on what values you are interested in you can adjust what sentences you
                // wish to get and ignore all other sentences.
                match values {
                    GpsSentence::GGA(s) => {
                        let mut data = latest_data_clone.lock().unwrap();
                        data.utc = s.utc;
                        data.lat = s.lat;
                        data.long = s.long;
                        data.hdop = s.hdop;
                        data.alt = s.msl_alt;

                        data.gga = true;
                    },
                    GpsSentence::GSA(s) => {
                        let mut data = latest_data_clone.lock().unwrap();
                        data.pdop = s.pdop;
                        data.vdop = s.vdop;

                        data.gsa = true;
                    },
                    GpsSentence::VTG(s) => {
                        let mut data = latest_data_clone.lock().unwrap();
                        data.heading = s.magnetic_course;
                        data.speed = match s.speed_kph {
                            Some(n) => {
                                Some(n * 1000f32 / 3600f32)  // kph to m/s
                            },
                            None => None,
                        };
                    
                        data.vtg = true;
                    },
                    
                    _ => {
                        ()
                    }
                
                }

                {
                    if !*async_run_clone.lock().expect("could not lock mutex") {
                        break;
                    }
                }
            }
        });


        GPS {
            gps,
            async_run,
            latest_data,
        }
    }

    pub fn get_data(&mut self) -> Option<GpsData> {
        let mut data = self.latest_data.lock().unwrap();
        
        //println!("{:?}", data);
        if !data.ready() {
            return None;
        }

        let res = data.toUser();

        *data = Data::new();

        return Some(res);
    }
}