# PLOP
code base for the SARP 2022-2023 payload

# STM32 + Mbed OS Setup
## Development Setup
development must occur on a STM32 based board, depending on your board you may need an external programmer, and additional drivers installed on your computer

To start download and install [mbed studio](https://os.mbed.com/studio/)
Then clone this repository with
```
git clone https://github.com/explosion33/PLOP.git
```

The repository does not contain a copy of mbed-os so make sure you download one which sould be possible from mbed studio itself.
For this code base we are currently using mbed-os version 6.2.1


# Raspberry Pi + Rust Setup
## Development Setup
development must occur on a raspberry pi, or pi-like device. This can be streamlines with VSCodes' remote editing features
### Rust Setup
```curl https://sh.rustup.rs -sSf | sh```

and follow the setup instructions

```
rustc –version
cargo –version
```


### Setup
```
git clone https://github.com/explosion33/PLOP.git
```

to run the debug code
```
cargo run
```

### build for release
```
cargo build --release
```

### Compiling for raspberrypi zero 2W
```
rustup target add arm-unknown-linux-musleabi
cargo run --target=arm-unknown-linux-musleabi
```
