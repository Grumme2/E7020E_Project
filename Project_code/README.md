# LTU ES

This is an example of how to use radio to publish data to a central server.

An example application that uses RPC from Thingsboard to increment a counter
and publish that counter to Thingsboard is available at `src/main.rs`.

## Debug
```
openocd -f openocd.cfg
cargo run
```

## Flashing
```
cargo objcopy --release --bin comm-module -- -O binary test.bin
dfu-util --device 0483:df11 --alt 0 --dfuse-address 0x08000000:leave --download test.bin
```
