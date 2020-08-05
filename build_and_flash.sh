cargo build --release
arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/release/stm32_blink stm32_blink.bin
st-flash write stm32_blink.bin 0x8000000
