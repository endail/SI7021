# SI7021

[![Build on Raspberry Pi](https://github.com/endail/SI7021/actions/workflows/buildcheck.yml/badge.svg)](https://github.com/endail/SI7021/actions/workflows/buildcheck.yml) [![cppcheck](https://github.com/endail/SI7021/actions/workflows/cppcheck.yml/badge.svg)](https://github.com/endail/SI7021/actions/workflows/cppcheck.yml)

- Use with Raspberry Pi
- Requires [lgpio](http://abyz.me.uk/lg/index.html)
- Code tested inside [virtual Raspberry Pi Zero/3/4 environments](.github/workflows/buildcheck.yml) on GitHub

## Example

```cpp
SI7021::SI7021 sensor;
sensor.setup();
sensor.refresh();

std::cout 
    << "Temperature: "
    << sensor.getTemperature() << " C, "
    << "Humidity: "
    << sensor.getHumidity() << " %RH"
    << std::endl;
```

## Documentation

- `SI7021( int dev, int addr )`. Constructor taking `dev` identifying which I2C device to use (default is device 1), and `addr` identifying which `I2C` address to use (default is 0x40).

- `void setup( )`. Setup and connect to the device.

- `void close( )`. Disconnect from the device.

- `void refresh( )`. Get a humidity and temperature sample from the sensor.

- `double getTemperature( )`. Returns the last read temperature in degrees celsius.

- `double getHumidity( )`. Returns the last read relative humidity as a percentage.

- `void reset( )`. Reset the device.

- `void resetSettings( )`. Reset UserRegister1 register (ie. measurement resolution and heater status).

- `void resetHeater( )`. Reset HeaterControlRegister register (ie. heater power).

- `std::uint8_t getMeasurementResolution( )`. Returns the measurement resolution from the device, between 0 and 3.

- `void setMeasurementResolution( std::uint8_t res )`. Sets the measurement resolution, between 0 and 3.

- `VddStatus getVddStatus( )`. Returns `VddStatus::OK` if voltage level is sufficient, otherwise `VddStatus::LOW`.

- `HeaterStatus getHeaterStatus( )`. Returns `HeaterStatus::ENABLED` if the heater is enabled, or `HeaterStatus::DISABLED` if disabled.

- `void setHeaterStatus( HeaterStatus status )`. Enable or disable the heater.

- `std::uint8_t getHeaterPower( )`.  Returns the heater power, between 0 and 15.

- `void setHeaterPower( std::uint8_t power )`. Set the heater power, between 0 and 15.

- `SerialNumber getSerialNumber( )`. Gets the serial number of the device. `SerialNumber` is a typedef for a `std::uint64_t`.

- `DeviceId getDeviceId( )`. Gets the device id.

| identifier               | description        | devIdToString      |
| ------------------------ | ------------------ | ------------------ |
| `DeviceId::ENG_SAMPLE_1` | engineering sample | engineering sample |
| `DeviceId::ENG_SAMPLE_2` | engineering sample | engineering sample |
| `DeviceId::SI7013`       | SI7013             | Si7013             |
| `DeviceId::SI7020`       | SI7020             | Si7020             |
| `DeviceId::SI7021`       | SI7021             | Si7021             |
| `DeviceId::UNKNOWN`      | unknown device     | unknown            |

- `FirmwareRevision getFirmwareRevision( )`. Gets the device firmware revision.

| identifier                  | description      | fwRevToString |
| --------------------------- | ---------------- | ------------- |
| `FirmwareRevision::REV_1_0` | revision 1.0     | 1.0           |
| `FirmwareRevision::REV_2_0` | revision 2.0     | 2.0           |
| `FirmwareRevision::UNKNOWN` | unknown firmware | unknown       |

- `const char* const devIdToString( DeviceId id )`. Returns a C string representing the device id.

- `const char* const fwRevToString( FirmwareRevision rev )`. Returns a C string representing the firmware revision.



