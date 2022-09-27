#![allow(dead_code)]

use rppal::gpio::Gpio;
use rppal::gpio::OutputPin;
use rppal::gpio::InputPin;

use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::thread;

pub struct Igniter {
    fire_pin: Arc<Mutex<OutputPin>>,
    cont_pin: InputPin,
    //gpio: Gpio
}

impl Igniter {
    pub fn new(fire_pin: u8, cont_pin: u8) -> Igniter {
        let gpio = Gpio::new().expect("unable to create GPIO");
        let fp: OutputPin = gpio.get(fire_pin).expect("could not initialize fire pin").into_output();
        let fire = Arc::new(Mutex::new(fp));
        let cont: InputPin = gpio.get(cont_pin).expect("could not initialize cont pin").into_input();
        Igniter{fire_pin: fire, cont_pin: cont}//, gpio: gpio}
    }
    pub fn has_continuity(&mut self) -> bool {
        self.cont_pin.is_high()
    }

    fn fire_helper(pin: &mut OutputPin) {
        pin.set_high();
        thread::sleep(Duration::from_millis(500));
        pin.set_low();
    }

    pub fn fire(&mut self) {
        // derefence Arc, lock mutex, call fire helper, without 
        Igniter::fire_helper(&mut *self.fire_pin.lock().unwrap());
    }

    pub fn fire_async(&mut self) -> thread::JoinHandle<()> {
        // Arc creates a thread safe pointer
        //  creates a smart pointer that keeps track of the number of references
        // Mutex creates a locking system for safe thread acces
        //  .lock() locks the memory so only one location can access it at a time

        // here we clone the Arc .. i.e. create a copy of the pointer
        let p = Arc::clone(&self.fire_pin);
        thread::spawn(move || {
            // we move the copy into the thead, dereference the pointer,
            // lock the mutex, and create a mutable reference to the pin
            let p = &mut *p.lock().unwrap();

            // we then pass the pin to our helper function which initiates the fire
            Igniter::fire_helper(p);
        })
    }
}