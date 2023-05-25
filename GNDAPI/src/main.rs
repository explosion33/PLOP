use crate::api::start_api;
mod api;

use crate::protocol::{RocketData, decode_stream, DATA_STREAM_SIZE};
mod protocol;

use ArmlabRadio::radio_serial::{Radio, prompt_port};


use std::{thread, usize};
use std::time::{Duration, Instant};
use std::sync::{Arc, Mutex};


fn radio(arc_data: api::TData) {
    let port = prompt_port();

    let mut radio = Radio::new(&port).expect("Error Creating Radio");
    radio.set_power(14f32).expect("error setting power");

    let mut iter_np: u16 = 0;
    
    let mut last_time: f32 = 0f32;

    let mut total_restarts: i32 = 0;

    loop {
        if iter_np == 50 {
            total_restarts += 1;
            iter_np = 0;

            radio.soft_reset().unwrap();

            drop(radio);
                              
            radio = loop {
                match Radio::new_bare(&port) {
                    Ok(n) => {break n;},
                    Err(_) => {},
                };
            };

            radio.set_power(14f32).expect("error setting power");

            //thread::sleep(Duration::from_millis(30));
            println!("reset radio, re-initialized serial coms");
        } 

        let mut data = match arc_data.lock() {
            Ok(n) => n,
            Err(_) => {
                println!("could not lock mutex");
                continue;
            } 
        };

        // handle thread quit
        if !data.is_alive {
            return ();
        }

        /* 
        // handle commands
        for command in data.cmds.iter() {
            let (cmd, arg) = command;

            println!("got cmd {}, with args {}", cmd, arg);

            let mut buf: [u8; 5] = [0u8; 5];
            match cmd.as_str() {
                "test" => {
                    buf[0] = 2;
                    
                    let f = arg.to_le_bytes();
                    buf[1] = f[0];
                    buf[2] = f[1];
                    buf[3] = f[2];
                    buf[4] = f[3];

                }
                _ => {
                    println!("unknown command");
                }
            }
            
            match radio.transmit(&buf) {
                Ok(_) => {},
                Err(n) => {
                    radio.sync(10);
                    println!("transmit error: {:?} | skipping command: {}", n, cmd);
                }
            };
        }
        data.cmds.clear();
        */

        /*
        // downlink
        let mut handle_packet = || -> () {
            tot_packets += 1;
            // get data stream
            let buf = match radio.get_packet() {
                Ok(n) => {
                    if n.len() == 0 {
                        return;
                    }
                    n
                },
                Err(n) => {
                    radio.sync(10);
                    println!("Error getting packet: {:?}", n);
                    return;
                }
            };

            let buf: [u8; DATA_STREAM_SIZE] = match buf.try_into() {
                Ok(n) => n,
                Err(n) => {
                    println!("Error | expected length {} got {} ", DATA_STREAM_SIZE, n.len());
                    return;
                }
            };

            let rec_data: RocketData = match decode_stream(buf) {
                Ok(n) => n,
                Err(n) => {
                    println!("Error decoding stream | {}", n);
                    return;
                }
            };

            good_packets += 1;

            let time: f32 = rec_data.time as f32 / 1000f32;

            data.altitude.push((time, rec_data.altitude)); 
            data.orx.push((time, rec_data.orx));
            data.ory.push((time, rec_data.ory));
            data.orz.push((time, rec_data.orz));
            data.lat.push((time, rec_data.lat));
            data.long.push((time, rec_data.long));
            data.fix.push((time, rec_data.fix as f32));
            data.quality.push((time, rec_data.quality as f32));
            data.cont_droug.push((time, if rec_data.cont1 {1f32} else {0f32}));
            data.cont_main.push((time, if rec_data.cont2 {1f32} else {0f32}));

        };

        handle_packet();

        drop(data);

        // if we are unable to parse a data stream we continue; this skips the heartbeat section alltogether
        
        if start_time.elapsed() >= Duration::from_millis(1000) {
            println!("{} / {} | {}%", good_packets, tot_packets, (good_packets*100/tot_packets));
            if tot_packets > 0 && (good_packets*100/tot_packets) >= 50 {
                iter += 1;
                // transmit heartbeat
                println!("sending heartbeat {}", iter);
                
                
                match radio.transmit(&[1, 1, 1, 1, 1]) {
                    Ok(_) => {},
                    Err(n) => {
                        radio.sync(10);
                        println!("transmit error: {:?} | skipping heartbeat", n);
                    }
                }
                thread::sleep(Duration::from_millis(100));
            }
            else {
                println!("skipping heartbeat");
            }

            start_time = Instant::now();
            good_packets = 0;
            tot_packets = 0;
        }
        */
        
        // give api a chance to aquire mutex lock

        let buf = match radio.get_packet() {
            Ok(n) => {
                if n.len() == 0 || n.len() != 32 {
                    iter_np += 1;
                    drop(data);
                    thread::sleep(Duration::from_millis(20));
                    continue;
                }
                n
            },
            Err(n) => {
                radio.sync(10).ok();
                println!("Error getting packet: {:?}", n);
                continue;
            }
        };


        let time: f32 = f32::from_le_bytes(match (&buf[0..4]).try_into() {
            Ok(n) => n,
            Err(n) => {
                //println!("Error | expected length {} got {} ", DATA_STREAM_SIZE, n.len());
                continue;
            }
        });

        let alt: f32 = f32::from_le_bytes(match (&buf[4..8]).try_into() {
            Ok(n) => n,
            Err(n) => {
                //println!("Error | expected length {} got {} ", DATA_STREAM_SIZE, n.len());
                continue;
            }
        });

        let balt: f32 = f32::from_le_bytes(match (&buf[8..12]).try_into() {
            Ok(n) => n,
            Err(n) => {
                //println!("Error | expected length {} got {} ", DATA_STREAM_SIZE, n.len());
                continue;
            }
        });

        let vel: f32 = f32::from_le_bytes(match (&buf[12..16]).try_into() {
            Ok(n) => n,
            Err(n) => {
                //println!("Error | expected length {} got {} ", DATA_STREAM_SIZE, n.len());
                continue;
            }
        });

        let w: f32 = f32::from_le_bytes(match (&buf[16..20]).try_into() {
            Ok(n) => n,
            Err(n) => {
                //println!("Error | expected length {} got {} ", DATA_STREAM_SIZE, n.len());
                continue;
            }
        });

        let x: f32 = f32::from_le_bytes(match (&buf[20..24]).try_into() {
            Ok(n) => n,
            Err(n) => {
                //println!("Error | expected length {} got {} ", DATA_STREAM_SIZE, n.len());
                continue;
            }
        });

        let y: f32 = f32::from_le_bytes(match (&buf[24..28]).try_into() {
            Ok(n) => n,
            Err(n) => {
                //println!("Error | expected length {} got {} ", DATA_STREAM_SIZE, n.len());
                continue;
            }
        });

        let z: f32 = f32::from_le_bytes(match (&buf[28..32]).try_into() {
            Ok(n) => n,
            Err(n) => {
                //println!("Error | expected length {} got {} ", DATA_STREAM_SIZE, n.len());
                continue;
            }
        });

        if time != last_time {
            data.altitude.push((time, alt));
            data.balt.push((time, balt));
            data.vel.push((time, vel));

            data.w.push((time, w));
            data.x.push((time, x));
            data.y.push((time, y));
            data.z.push((time, z));

            last_time = time;
        }


        drop(data);

        println!("time: {}, alt: {}, balt: {}, vel: {}, or: [{}, {}, {}, {}] || fails: {}",
            time,
            alt,
            balt,
            vel,
            w,
            x,
            y,
            z,
            total_restarts
        );


        iter_np = 0;

        thread::sleep(Duration::from_millis(100));
    } 

}

fn sim(arc_data: api::TData) {

    let time: Vec<f32> = vec![];
    let alt:  Vec<f32> = vec![];

    let mut i: usize = 0;
    let mut start: bool = false;


    loop {
        let mut data = match arc_data.lock() {
            Ok(n) => n,
            Err(_) => {
                println!("could not lock mutex");
                continue;
            } 
        };

        // handle thread quit
        if !data.is_alive {
            return ();
        }

        // handle commands
        for command in data.cmds.iter() {
            let (cmd, arg) = command;

            println!("got cmd {}, with args {}", cmd, arg);

            match cmd.as_str() {
                "sim" => {
                    if arg == &1.0 {
                        start = true;
                    }
                }
                _ => {}
            }
        }
        data.cmds.clear();

        if (start) {
            data.altitude.push((time[i], alt[i])); 
            //data.orx.push((time, rec_data.orx));
            //data.ory.push((time, rec_data.ory));
            //data.orz.push((time, rec_data.orz));
            //data.lat.push((time, rec_data.lat));
            //data.long.push((time, rec_data.long));
            //data.fix.push((time, rec_data.fix as f32));
            //data.quality.push((time, rec_data.quality as f32));
            //data.cont_droug.push((time, if rec_data.cont1 {1f32} else {0f32}));
            //data.cont_main.push((time, if rec_data.cont2 {1f32} else {0f32}));

            drop(data);


            thread::sleep(Duration::from_millis(200));
            i += 1;

            if (i >= time.len()) {
                i = 0;
                start = false;
            }
        }
    } 

}


fn main() {
    let data = api::Data::new();
    let thread_data: api::TData = Arc::new(Mutex::new(data));
    let collect = Arc::clone(&thread_data);


    
    // move serial radio handler to thread with shared data struct
    let handle = thread::spawn(move || {
        println!("setting up thread");
        radio(collect);
    });
    
    // move api to thread with same shared data struct
    println!("starting api");
    let handle2 = thread::spawn(move || {
        start_api(thread_data);
    });

    // check if either thread quits and terminate the program if they do
    // radio will panic with a radio error
    // api will close once a quit command is sent
    loop {
        if handle.is_finished() || handle2.is_finished() {
            println!("one of the threads closed, terminating");
            handle.join().unwrap();
            handle2.join().unwrap();
            return
        }
    }

}
