RF Sensor based on NRF24L01+, BME280 modules and ATmega328P microcontroller. Second generation of sensor design, previous one was based on AM2302 (wired DHT22) which draws ~100x more current in stand-by mode.
Written purely for WinAVR, not Arduino!
This sensor is designed to consume very little power and work on a coin battery (ex: CR2025) for months, it sends sensor readings each hour.

Specifications:
- Power supply:  1.8V .. 3.6V
- Stand-by current: ~2.04 uA
- Active transmitter mode current: ~10mA

Collected data:
- Temperature
- Humidity
- Pressure
- Battery level

Circuit diagram:
![circuit diagram](/docs/SensorV2Schematic.png?raw=true "Sensor circuit diagram")

Fuse Settings for ATmega328P:
![fuse settings](/docs/FuseSettings.png?raw=true "Fuse Settings")

BME/BMP280 breakout board modification to work only with 3.3V:
![breakout board](/docs/BME280Breakout.png?raw=true "Breakout Board")

Prototype PCB:
![prototype pcb](/docs/PrototypePCB.png?raw=true "Prototype PCB")