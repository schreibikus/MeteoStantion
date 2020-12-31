# MeteoStantion

Simple Meteo Stantion based on NodeMCU v2 board and BME280 sensor.

For compilation is used the latest *[ESP8266 RTOS SDK](https://github.com/espressif/ESP8266_RTOS_SDK)* version.

Setup toolchain:

wget https://dl.espressif.com/dl/xtensa-lx106-elf-gcc8_4_0-esp-2020r3-linux-amd64.tar.gz
tar xf xtensa-lx106-elf-gcc8_4_0-esp-2020r3-linux-amd64.tar.gz
export PATH=$PWD/xtensa-lx106-elf/bin:$PATH

Setup SDK:

git clone https://github.com/espressif/ESP8266_RTOS_SDK.git
export IDF_PATH=$PWD/ESP8266_RTOS_SDK

Compile application:

cd MeteoStation
make all


