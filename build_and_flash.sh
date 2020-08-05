cargo build --release
arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/release/stm32_test stm32_test.bin
st-flash write stm32_test.bin 0x8000000
