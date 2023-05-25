pub const DATA_STREAM_SIZE: usize = 34;

#[derive(Debug)]
pub struct RocketData {
    pub time: u32,
    pub altitude: f32,
    pub orx: f32,
    pub ory: f32,
    pub orz: f32,
    pub lat: f32,
    pub long: f32,
    pub speed: f32,
    pub fix: u8,
    pub quality: u8,
    pub cont1: bool,
    pub cont2: bool,
}

impl PartialEq for RocketData {
    fn eq(&self, other: &Self) -> bool {
        let b_time = self.time == other.time;
        let b_altitude = self.altitude == other.altitude;
        let b_orx = self.orx == other.orx;
        let b_ory = self.ory == other.ory;
        let b_orz = self.orz == other.orz;
        let b_lat = self.lat == other.lat;
        let b_long = self.long == other.long;
        let b_speed = self.speed == other.speed;
        let b_fix = self.fix == other.fix;
        let b_quality = self.quality == other.quality;
        let b_cont1 = self.cont1 == other.cont1;
        let b_cont2 = self.cont2 == other.cont2;

        return b_time && b_altitude && b_orx && b_ory && b_orz && b_lat && b_long && b_speed && b_fix && b_quality && b_cont1 && b_cont2;
    }
}

pub fn encode_stream(data: &RocketData) -> Result<[u8; DATA_STREAM_SIZE], String> {
    let mut buf: Vec<u8> = vec![];

    buf.extend_from_slice(&data.time.to_be_bytes());
    buf.extend_from_slice(&data.altitude.to_be_bytes());
    buf.extend_from_slice(&data.orx.to_be_bytes());
    buf.extend_from_slice(&data.ory.to_be_bytes());
    buf.extend_from_slice(&data.orz.to_be_bytes());
    buf.extend_from_slice(&data.lat.to_be_bytes());
    buf.extend_from_slice(&data.long.to_be_bytes());
    buf.extend_from_slice(&data.speed.to_be_bytes());


    let mut fix_qual: u8 = data.quality << 4;
    fix_qual += data.fix & 0b00001111;
    buf.push(fix_qual);

    let mut conts: u8 = 0;
    if data.cont1 {
        conts += 1;
    }
    if data.cont2 {
        conts += 2;
    }

    buf.push(conts);

    match buf.as_slice().try_into() {
        Ok(n) => Ok(n),
        Err(_) => Err("Error converting vec to slice".to_string()),
    }
}

pub fn decode_stream(buf: [u8; DATA_STREAM_SIZE]) -> Result<RocketData, String> {
    let time: u32 = u32::from_be_bytes(match buf[0..4].try_into(){
        Ok(n) => n,
        Err(_) => {return Err("error converting time to u32".to_string())},
    });

    let altitude: f32 = f32::from_be_bytes(match buf[4..8].try_into() {
        Ok(n) => n,
        Err(_) => {return Err("error converting altitude to f32".to_string())},
    });
    let orx: f32 = f32::from_be_bytes(match buf[8..12].try_into() {
        Ok(n) => n,
        Err(_) => {return Err("error converting orx to f32".to_string())},
    });
    let ory: f32 = f32::from_be_bytes(match buf[12..16].try_into() {
        Ok(n) => n,
        Err(_) => {return Err("error converting ory to f32".to_string())},
    });
    let orz: f32 = f32::from_be_bytes(match buf[16..20].try_into() {
        Ok(n) => n,
        Err(_) => {return Err("error converting orz to f32".to_string())},
    });
    let lat: f32 = f32::from_be_bytes(match buf[20..24].try_into() {
        Ok(n) => n,
        Err(_) => {return Err("error converting lat to f32".to_string())},
    });
    let long: f32 = f32::from_be_bytes(match buf[24..28].try_into() {
        Ok(n) => n,
        Err(_) => {return Err("error converting long to f32".to_string())},
    });
    let speed: f32 = f32::from_be_bytes(match buf[28..32].try_into() {
        Ok(n) => n,
        Err(_) => {return Err("error converting speed to f32".to_string())},
    });

    // 32: 0000 0000
    //     qual fix
    let fix: u8 = buf[32] & 0b00001111;// first 4 (least significant) bits of 29
    let quality: u8 = buf[32] >> 4;// last 4 bits of 29

    // 00000   0   0
    //         2   1
    let cont1: bool = buf[33] & 1 == 1;        // first (lsb) of 30
    let cont2: bool = (buf[33] >> 1) & 1 == 1; // second (lsb) of 30
    


    Ok(RocketData {time, altitude, orx, ory, orz, lat, long, speed, fix, quality, cont1, cont2})
}

#[cfg(test)]
mod tests {
    use rand::Rng;
    use crate::protocol::{RocketData, encode_stream, decode_stream};

    fn generate_random_data() -> RocketData {
        let mut rng = rand::thread_rng();
    
        let time: u32 = rng.gen_range(0u32..1000u32);
        let altitude: f32 = rng.gen_range(-100f32..100f32);
        let orx: f32 = rng.gen_range(-100f32..100f32);
        let ory: f32 = rng.gen_range(-100f32..100f32);
        let orz: f32 = rng.gen_range(-100f32..100f32);
        let lat: f32 = rng.gen_range(-100f32..100f32);
        let long: f32 = rng.gen_range(-100f32..100f32);
        let speed: f32 = rng.gen_range(-100f32..100f32);
        let fix: u8 = rng.gen_range(0u8..5u8);
        let quality: u8 = rng.gen_range(0u8..5u8);
        let cont1: bool = rng.gen();
        let cont2: bool = rng.gen();
    
        RocketData { time, altitude, orx, ory, orz, lat, long, speed, fix, quality, cont1, cont2 }
    }
    

    #[test]
    fn rand_enc_dec() {
        let data: RocketData = generate_random_data();

        let new_data = decode_stream(encode_stream(&data).expect("error encoding stream")).expect("error decoding stream");

        assert_eq!(data, new_data);
    }

    #[test]
    fn many_random() {
        for _ in 0..50 {
            rand_enc_dec();
        }
    }

    #[test]
    fn order_enc_dec() {
        let data: RocketData = RocketData {
            time: 1u32,
            altitude: 2f32,
            orx: 3f32,
            ory: 4f32,
            orz: 5f32, 
            lat: 6f32, 
            long: 7f32,
            speed: 8f32,
            fix: 9u8,
            quality: 10u8,
            cont1: false,
            cont2: true,
        };

        println!("{:?}", encode_stream(&data).expect("error encoding stream"));

        let new_data = decode_stream(encode_stream(&data).expect("error encoding stream")).expect("error decoding stream");

        assert_eq!(data, new_data);

    }
}