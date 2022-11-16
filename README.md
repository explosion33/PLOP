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
git clone https://github.com/explosion33/RocketController.git
```

to run the debug code
```
cargo run
```

### build for release
```
cargo build --release
```

## Using the latest release
download the latest [release](https://github.com/explosion33/RocketController/releases) onto raspbery pi or pi-like device

edit `rocket.toml` to change api server settings and `flight.ini` to adjust in-flight calculation settings

run with `./path/to/rocket_controller`

'test'
