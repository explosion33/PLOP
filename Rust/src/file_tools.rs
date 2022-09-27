#![allow(dead_code)]

use std::fs::{File, OpenOptions, read_to_string};
use std::collections::HashMap;
use std::io::{Write, ErrorKind};

pub fn parse_ini(path: &str) -> Result<HashMap<String, String>, &str>{
    let contents = match read_to_string(path) {
        Ok(n) => {n},
        Err(_) => {return Err("could not read file")},
    };
    

    let mut data: HashMap<String, String> = HashMap::new();

    for line in contents.split("\n") {
        let vals: Vec<&str> = line.split("=").collect();

        if vals.len() == 2 {
            data.insert(
                vals[0].trim().to_string(),
                vals[1].trim().to_string()
            );
        }
    }
    return Ok(data);
}

pub struct Logger {
    file: File,
    is_line: bool,
}


impl Logger {
    pub fn new(path: &str) -> Logger {
        
        let file = match OpenOptions::new().write(true).open(path) {
            Ok(n) => {n},
            Err(n) => {
                if n.kind() != ErrorKind::NotFound {
                    panic!("Fatal | could not open file");
                }
                File::create(path).expect("Fatal | file not found, and could not be created");
                OpenOptions::new().write(true).open(path).expect("Fatal | file created, but could not be opened")
            },
        };
        Logger {file, is_line: false}
    }

    pub fn write<T: ToString>(&mut self, data: &T) -> Result<(), String> {
        let mut data: String = data.to_string();

        if self.is_line {
            data = ", ".to_string() + data.as_str();
        }
        else {
            self.is_line = true;
        }

        match self.file.write_all(data.as_bytes()) {
            Ok(_) => {},
            Err(n) => {
                println!("{:#?}", n);
                return Err("could not write to file".to_string())}
        };

        Ok(())
    }

    pub fn write_vec<T: ToString>(&mut self, data: &Vec<T>) -> Result<(), String> {
        let mut out: String = "[".to_string();
        for i in 0..data.len()-1 {
            out += data[i].to_string().as_str();
            out += ", ";
        }
        out += data[data.len()-1].to_string().as_str();
        out += "]";
        
        self.write(&out)
    }

    pub fn end_cycle(&mut self) -> Result<(), String> {
        match self.file.write_all("\n".as_bytes()) {
            Ok(_) => {},
            Err(_) => {return Err("could not write to file".to_string())}
        };
        match self.file.sync_data() {
            Ok(_) => {},
            Err(_) => {return Err("could not sync file".to_string())}
        };

        self.is_line = false;

        Ok(())
    }
}