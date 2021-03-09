# MAVESP32
The source codes of MAVESP32 is from [MAVESP8266](https://github.com/tridge/mavesp8266)

It has been modified for ESP32 chipset usage. Check "kewu" marker for modifying history

The lib code (src/mavesp8266.*) is placed in .\Documents\Arduino\libraries\mavesp8266\

The ino code (example/MAVesp32.ino) is placed in .\Documents\Arduino\MAVesp32

The codes is modified on Arduino IDE 1.8.13 and testing on an [ublox NINA-W102](https://www.u-blox.com/en/product/nina-w10-series-open-cpu)

## Device setting in the Arduino IDE
In the Tools -> Board menu, select “ESP32 Dev Module” and then select the following; 

•   Flash Mode: “DIO” 

•   Flash Frequency: “40 MHz”

•   Flash Size: “2 MB (16 Mb)” 

•   Upload Speed “921600” 

•   Core Debug Level “Debug” (optional) 

## Wiring
[ESP32 pico kit](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-pico-kit.html) can be used for communicating

[ESP8266 wifi telemetry](https://ardupilot.org/copter/docs/common-esp8266-telemetry.html) is shown how to wire it up

ESP32 Pico TXD (Pin 13) <--> RXD PX4

ESP32 Pico RXD (12) <--> TXD PX4

ESP32 Pico 5V (Pin 19) <--> 5V PX4

ESP32 Pico GND (Pin20) <--> GND PX4
