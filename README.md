# PLOP
code base for the SARP 2022-2023 payload

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
