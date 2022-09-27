use crate::igniter::Igniter;
mod igniter;

use crate::baro::Baro;
mod baro;

use crate::indicator::Indicator;
mod indicator;

use crate::api::start_api;
mod api;

use crate::file_tools::{parse_ini, Logger};
mod file_tools;

use std::thread;
use std::time::Duration;
use std::sync::{Arc, Mutex};
use std::time::SystemTime;


/*const M_FIRE: u8 = 23;
const M_CONT: u8 = 22;

const D_FIRE: u8 = 27;
const D_CONT: u8 = 17;

const BARO_CONFIG_PATH: &str = "baro.conf";

//rolling average
const WINDOW_SIZE: usize = 35;
//number of points for baseline average
const BASELINE_ITER: usize = 150;
//change in meters, to count as a significant change
//should be adjusted to overcome noise
const SIG_ALT_CHANGE: f32 = 1.1f32;
const ITER_ABOVE_SIG: u8 = 50;

const MAIN_DEPLOY_ALT: f32 = 199f32;
*/

#[derive(Clone)]
#[allow(non_snake_case)]
struct Settings {
    pub M_FIRE: u8,
    pub M_CONT: u8,
    pub D_FIRE: u8,
    pub D_CONT: u8,
    pub BUZEER: u8,
    pub STATUS: u8,

    pub BARO_CONFIG_PATH: String,

    pub WINDOW_SIZE: usize,
    pub BASELINE_ITER: usize,

    pub SIG_ALT_CHANGE: f32,
    pub ITER_ABOVE_SIG: u8,
    pub MAIN_DEPLOY_ALT: f32,
}

impl Settings {
    pub fn new() -> Settings{
        let settings = parse_ini("flight.ini").expect("flight.ini not found");

        let vals = ["MAIN_FIRE", "MAIN_CONT", "DROUG_FIRE", "DROUG_CONT", "BUZZER_PIN", "STATUS_PIN", "BARO_CONFIG_PATH", "WINDOW_SIZE", "BASELINE_ITER", "SIG_ALT_CHANGE", "ITER_ABOVE_SIG", "MAIN_DEPLOY_ALT"];

        for &val in &vals {
            if !settings.contains_key(val) {
                panic!("Error | missing field \"{}\" in flight.ini", val);
            }
        }

        Settings {
            M_FIRE: settings["MAIN_FIRE"].parse::<u8>().expect("invalid setting"),
            M_CONT: settings["MAIN_CONT"].parse::<u8>().expect("invalid setting"),
            D_FIRE: settings["DROUG_FIRE"].parse::<u8>().expect("invalid setting"),
            D_CONT: settings["DROUG_CONT"].parse::<u8>().expect("invalid setting"),
            BUZEER: settings["BUZZER_PIN"].parse::<u8>().expect("invalid setting"),
            STATUS: settings["STATUS_PIN"].parse::<u8>().expect("invalid setting"),
            BARO_CONFIG_PATH: settings["BARO_CONFIG_PATH"].to_string(),
            WINDOW_SIZE: settings["WINDOW_SIZE"].parse::<usize>().expect("invalid setting"),
            BASELINE_ITER: settings["BASELINE_ITER"].parse::<usize>().expect("invalid setting"),
            SIG_ALT_CHANGE: settings["SIG_ALT_CHANGE"].parse::<f32>().expect("invalid setting"),
            ITER_ABOVE_SIG: settings["ITER_ABOVE_SIG"].parse::<u8>().expect("invalid setting"),
            MAIN_DEPLOY_ALT: settings["MAIN_DEPLOY_ALT"].parse::<f32>().expect("invalid setting"),
        }
    }
}


fn pre_flight(settings: &Settings) -> bool {
    // initiate shared memory
    let data = api::Data::new();
    let thread_data: api::TData = Arc::new(Mutex::new(data));
    let collect = Arc::clone(&thread_data); //created then moved
    let cloned_settings = settings.clone();


    // start and wait for thread to finish
    let handle = thread::spawn(move || {
        println!("setting up thread");
        api_getter(&cloned_settings, collect);
    });

    println!("starting api");
    let is_armed = start_api(thread_data);
    println!("api closed");
    let _ = handle.join();
    println!("thread closed");

    return is_armed;
}

fn api_getter(settings: &Settings, thread_data: api::TData) {
    // sensors
    let mut barometer = Baro::new(settings.BARO_CONFIG_PATH.to_string().as_str());
    let mut main_ign = Igniter::new(settings.M_FIRE, settings.M_CONT);
    let mut droug_ign = Igniter::new(settings.D_FIRE, settings.D_CONT);

    // averaging vars
    let mut window: Vec<f32> = vec![];

    //main loop
    let start = SystemTime::now();

    println!("thread initialized, starting loop");

    loop {
        let dt = SystemTime::now().duration_since(start).expect("time went backwards").as_secs_f32();
        let mut data = thread_data.lock().unwrap();

        // check if rocket server has been closed
        if !data.is_alive {
            return ();
        }

        // sort through any push commands
        for command in data.cmds.iter() {
            let (cmd, val) = command;
            println!("cmd: {} [{}]", cmd, val);
            match cmd.as_str() {
                "config_baro" => {
                    barometer.configure(*val);
                },
                "fire" => {
                    match *val as i32 {
                        0 => {droug_ign.fire_async();},
                        1 => {main_ign.fire_async();},
                        _ => {},
                    };
                },
                _ => {},
            };
        }
        data.cmds.clear();

        // get barometer reading and adjust window
        match barometer.get_alt() {
            Ok(n) => {
                window.push(n);
                if window.len() >  settings.WINDOW_SIZE {
                    window.remove(0);
                }
            },
            Err(_) => {},
        };

        // if window is full, push the computed average to api server
        if window.len() == settings.WINDOW_SIZE {
            let mut avg: f32 = 0f32;
            for alt in window.iter() {
                avg += alt;
            }
            avg /= settings.WINDOW_SIZE as f32;

            data.altitude.push((dt, avg));
        }

        // other sensor data
        match barometer.get_pressure() {
            Ok(n) => {
                data.pressure.push((dt,n));
            }
            Err(_) => {},
        };

        match barometer.get_temp() {
            Ok(n) => {
                data.temperature.push((dt,n));
            }
            Err(_) => {},
        };

        // continuity sensor data
        data.cont_main.push((dt, main_ign.has_continuity() as i8 as f32));
        data.cont_droug.push((dt, droug_ign.has_continuity() as i8 as f32));

        //allow other threads a chance to lock mutex
        drop(data);
        thread::sleep(Duration::from_millis(50)); 
    }
}

fn detect_liftoff(settings: &Settings, logger: &mut Logger, barometer: &mut Baro, base_alt: &f32) {
    // detect significant upwards elevation change
    let mut window: Vec<f32> = vec![];
    let mut num_above_sig: u8 = 0;

    let start = SystemTime::now();
    loop {
        let dt = SystemTime::now().duration_since(start).expect("time went backwards").as_secs_f32();
        // rolling average altitudes
        match barometer.get_alt() {
            Ok(n) => {
                window.push(n);
                if window.len() > settings.WINDOW_SIZE {
                    window.remove(0);
                }
            },
            Err(_) => {},
        };

        // if our window is not full skip calculations
        // i.e. first WINDOW_SIZE - 1 values
        if window.len() != settings.WINDOW_SIZE {
            continue;
        }

        // average current window
        let mut avg: f32 = 0f32;
        for alt in window.iter() {
            avg += alt;
        }
        avg /= settings.WINDOW_SIZE as f32;

        let _ = logger.write(&dt);
        let _ = logger.write(&avg);
        match barometer.get_pressure() {
            Ok(n) => {let _ = logger.write(&n);},
            Err(_) => {},
        };
        match barometer.get_temp() {
            Ok(n) => {let _ = logger.write(&n);},
            Err(_) => {},
        };
        let _ = logger.end_cycle();


        // is significantly high
        if avg - base_alt >= settings.SIG_ALT_CHANGE {
            num_above_sig += 1;
            println!("{}/{}", num_above_sig, settings.ITER_ABOVE_SIG);
        }
        else {
            num_above_sig = 0;
        }

        // if above height threshold for given iterations
        if num_above_sig > settings.ITER_ABOVE_SIG {
            return ();
        }
    }
}

fn detect_apogee(settings: &Settings, logger: &mut Logger, barometer: &mut Baro, base_alt: &f32) {
    let mut window: Vec<f32> = vec![];

    let mut max_altitude = *base_alt;
    let mut num_above_sig: u8 = 0;

    let start = SystemTime::now();

    loop {
        let dt = SystemTime::now().duration_since(start).expect("time went backwards").as_secs_f32();
        match barometer.get_alt() {
            Ok(n)=>{
                window.push(n);
                if window.len() > settings.WINDOW_SIZE {
                    window.remove(0);
                }
            },
            Err(_) => {}
        };

        if window.len() < settings.WINDOW_SIZE {
            continue;
        }

        //=========
        let mut alt = 0f32;
        for val in window.iter() {
            alt += val;
        }
        alt /= settings.WINDOW_SIZE as i32 as f32;


        let _ = logger.write(&dt);
        let _ = logger.write(&alt);
        match barometer.get_pressure() {
            Ok(n) => {let _ = logger.write(&n);},
            Err(_) => {},
        };
        match barometer.get_temp() {
            Ok(n) => {let _ = logger.write(&n);},
            Err(_) => {},
        };
        let _ = logger.end_cycle();

        if alt > max_altitude {
            max_altitude = alt;
        }

        if max_altitude - alt >= settings.SIG_ALT_CHANGE {
            num_above_sig += 1;
            println!("{}/{}", num_above_sig, settings.ITER_ABOVE_SIG);
        }
        else {
            num_above_sig = 0;
        }

        // if above height threshold for given iterations
        if num_above_sig > settings.ITER_ABOVE_SIG {
            return ();
        }
    }
}

fn detect_main(settings: &Settings, logger: &mut Logger, barometer: &mut Baro) {
    let mut window: Vec<f32> = vec![];
    let mut num_above_sig: u8 = 0;

    let start = SystemTime::now();

    loop {
        let dt = SystemTime::now().duration_since(start).expect("time went backwards").as_secs_f32();
        match barometer.get_alt() {
            Ok(n)=>{
                window.push(n);
                if window.len() > settings.WINDOW_SIZE {
                    window.remove(0);
                }
            },
            Err(_) => {}
        };

        if window.len() < settings.WINDOW_SIZE {
            continue;
        }

        //=========
        let mut alt = 0f32;
        for val in window.iter() {
            alt += val;
        }
        alt /= settings.WINDOW_SIZE as i32 as f32;

        let _ = logger.write(&dt);
        let _ = logger.write(&alt);
        match barometer.get_pressure() {
            Ok(n) => {let _ = logger.write(&n);},
            Err(_) => {},
        };
        match barometer.get_temp() {
            Ok(n) => {let _ = logger.write(&n);},
            Err(_) => {},
        };
        let _ = logger.end_cycle();

        //=========

        if settings.MAIN_DEPLOY_ALT - alt >= settings.SIG_ALT_CHANGE {
            num_above_sig += 1;
            println!("{}/{}", num_above_sig, settings.ITER_ABOVE_SIG);
        }
        else {
            num_above_sig = 0;
        }

        // if above height threshold for given iterations
        if num_above_sig > settings.ITER_ABOVE_SIG {
            return ();
        }
    }
}

fn basic_log(settings: &Settings, logger: &mut Logger, barometer: &mut Baro) {
    let mut window: Vec<f32> = vec![];

    let start = SystemTime::now();

    loop {
        let dt = SystemTime::now().duration_since(start).expect("time went backwards").as_secs_f32();
        match barometer.get_alt() {
            Ok(n)=>{
                window.push(n);
                if window.len() > settings.WINDOW_SIZE {
                    window.remove(0);
                }
            },
            Err(_) => {}
        };

        if window.len() < settings.WINDOW_SIZE {
            continue;
        }

        //=========
        let mut alt = 0f32;
        for val in window.iter() {
            alt += val;
        }
        alt /= settings.WINDOW_SIZE as i32 as f32;

        let _ = logger.write(&dt);
        let _ = logger.write(&alt);
        match barometer.get_pressure() {
            Ok(n) => {let _ = logger.write(&n);},
            Err(_) => {},
        };
        match barometer.get_temp() {
            Ok(n) => {let _ = logger.write(&n);},
            Err(_) => {},
        };
        let _ = logger.end_cycle();
    }

}

fn main() {
    let settings = Settings::new();

    let mut buzzer = Indicator::new(settings.BUZEER);
    let mut led: Indicator = Indicator::new(settings.STATUS);

    buzzer.start(500, 1000, 5000);
    led.start(500, 1000, 5000);

    // pre-flight API
    let is_armed = pre_flight(&settings);

    if !is_armed {
        return ();
    }

    println!("initializing flight sensors");

    buzzer.start_inf(100, 200);
    led.start_inf(100, 200);

    let mut logger: Logger = Logger::new("flight.log");

    let mut main_ign = Igniter::new(settings.M_FIRE, settings.M_CONT);
    let mut droug_ign = Igniter::new(settings.D_FIRE, settings.D_CONT);    
    let mut barometer = Baro::new(settings.BARO_CONFIG_PATH.to_string().as_str());

    let mut base_alt: f32 = 0f32;
    let mut i = 0;
    while i < settings.BASELINE_ITER {
        match barometer.get_alt() {
            Ok(n) => {
                base_alt += n;
                i += 1;
                thread::sleep(Duration::from_millis(20));
            }
            Err(_) => {}
        }
    }
    base_alt /= i as f32;
    println!("base-alt: {}", base_alt);
    println!("sensors initialized, waiting for liftoff");

    buzzer.stop();
    led.stop();
    thread::sleep(Duration::from_millis(200)); //enough time for buzzer to stop
    // should be reworked to return a join call?
    // and managed so only one tone is running at a time

    buzzer.start_inf(1000, 1000);
    led.start_inf(1000, 1000);

    detect_liftoff(&settings, &mut logger, &mut barometer, &base_alt);

    buzzer.stop();
    led.stop();

    detect_apogee(&settings, &mut logger, &mut barometer, &base_alt);
    droug_ign.fire_async();

    buzzer.start(200, 200, 200);
    led.start(200, 200, 200);

    detect_main(&settings, &mut logger, &mut barometer);
    main_ign.fire_async();

    buzzer.start(200, 200, 200);
    led.start(200, 200, 200);

    basic_log(&settings, &mut logger, &mut barometer);


}
