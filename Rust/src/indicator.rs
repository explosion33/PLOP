#![allow(dead_code)]

use rppal::gpio::Gpio;
use rppal::gpio::OutputPin;

use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::time::SystemTime;
use std::thread;

pub struct Indicator {
    pin: Arc<Mutex<OutputPin>>,
    stop: Arc<Mutex<bool>>,
}

impl Indicator {
    pub fn new(pin: u8) -> Indicator {
        let gpio = Gpio::new().expect("unable to create GPIO");
        let pin: OutputPin = gpio.get(pin).expect("could not initialize pin").into_output();
        let pin = Arc::new(Mutex::new(pin));


        Indicator{pin, stop: Arc::new(Mutex::new(false))}
    }

    fn def(locked_pin: Arc<Mutex<OutputPin>>, duty_cycle: u64, cycle_len: u64, run_time: u64) {
        let mut pin = locked_pin.lock().unwrap();

        let start = SystemTime::now();

        loop {
            let dt = (SystemTime::now().duration_since(start).unwrap().as_secs_f64() * 1000f64) as u64;
            if dt >= run_time {
                pin.set_low();
                return;
            }

            pin.set_high();
            thread::sleep(Duration::from_millis(duty_cycle));
            pin.set_low();
            thread::sleep(Duration::from_millis(cycle_len - duty_cycle));

        }
    }

    fn indef(locked_pin: Arc<Mutex<OutputPin>>, duty_cycle: u64, cycle_len: u64, stop: Arc<Mutex<bool>>) {
        let mut pin = locked_pin.lock().unwrap();
        loop {
            if *stop.lock().unwrap() {
                pin.set_low();
                return;
            }


            pin.set_high();
            thread::sleep(Duration::from_millis(duty_cycle));
            pin.set_low();
            thread::sleep(Duration::from_millis(cycle_len - duty_cycle));

        }
    }

    pub fn start(&mut self, duty_cycle: u64, cycle_len: u64, run_time: u64) {
        let pin_copy = Arc::clone(&self.pin);

        thread::spawn(move || {
            Indicator::def(pin_copy, duty_cycle, cycle_len, run_time);
        });
    }

    pub fn start_inf(&mut self, duty_cycle: u64, cycle_len: u64) {
        {*self.stop.lock().unwrap() = false;}

        let pin_copy = Arc::clone(&self.pin);
        let stop_copy = Arc::clone(&self.stop);

        thread::spawn(move || {
            Indicator::indef(pin_copy, duty_cycle, cycle_len, stop_copy);
        });
    }

    pub fn stop(&mut self) {
        {*self.stop.lock().unwrap() = true;}
    }
}