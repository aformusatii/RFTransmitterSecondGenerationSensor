RF Sensor based on NRF24L01+ and BME280 modules. Second generation of sensor design, previous one was based on AM2302 (wired DHT22) which draws ~100x more current in stand-by mode.
Written purely for WinAVR, not Arduino!

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

Fuse Settings for Atmega328P:
![fuse settings](/docs/FuseSettings.png?raw=true "Fuse Settings")