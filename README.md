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
For this code base we are currently using mbed-os version 6.16.0

Once the files are setup, the program must be built and flashed, for our prototyping we are using the STM32 blackpill devboard which contains an STM32F401CCU6 processor. This same processor will be used on the final version of our flight computer so any steps working with the black pill will work with future boards as well

All instructions for seting up the development enviroment and flashing can be found [here](https://os.mbed.com/users/hudakz/code/Blackpill_Hello//file/a12913077fa8/README.md/)

Within these intructions I have found not using the custom BLACKPILL target and instead using one of the NUCLEO_F4XXX targets tends to work best. Also, I have had the most success flashing programs when using the SWD interface and the Nucleo ST-LINK programmer found on many NUCLEO dev boards

## Potential Problems when flashing
When flashing if you see that the ST-LINK starts flashing then stops, and the STM32 has not been reset and re-flashed
* Ensure all wires are properly connected
* Disconnect both ST-LINK and STM32 from power
* restart mbed studio
* reconnect STM32 to power, then ST-LINK
* flash

If problems persist:
* Attempt a clean build of the project


# Raspberry Pi + Rust Setup
Note: This Raspberyy pi and Rust version of the code has been deprecated and will not be used moving forward. It has been found that the STM32 system allows for better speeds and is generally easier to work with.
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
