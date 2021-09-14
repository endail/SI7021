// MIT License
//
// Copyright (c) 2020 Daniel Robertson
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef SI7021_H_FBEEB7BE_B548_47DA_966E_D5A42F85E5C0
#define SI7021_H_FBEEB7BE_B548_47DA_966E_D5A42F85E5C0

#include <bitset>
#include <cstdint>
#include <string>

namespace SI7021 {

typedef std::uint64_t SerialNumber;

enum class VddStatus : bool {
    OK = false,
    LOW = true
};

enum class HeaterStatus : bool {
    DISABLE = false,
    ENABLE = true
};

enum class DeviceId : std::uint8_t {
    ENG_SAMPLE_1 =  0x00,
    ENG_SAMPLE_2 =  0xff,
    SI7013 =        0x0d,
    SI7020 =        0x14,
    SI7021 =        0x15,
    UNKNOWN =       0x16 /* not defined in datasheet */
};

enum class FirmwareRevision : std::uint8_t {
    REV_1_0 =       0xff,
    REV_2_0 =       0x20,
    UNKNOWN =       0 /* not defined in datasheet */
};

struct UserRegister : std::bitset<8> {
    UserRegister() noexcept;
    UserRegister(const std::uint8_t v) noexcept;
    UserRegister(const std::bitset<8> bs) noexcept;
    virtual void resetSettings() = 0;
    std::uint8_t to_uint8_t() const noexcept;
};

struct UserRegister1 : public UserRegister {

    UserRegister1(const std::uint8_t bits = SI7021::DEFAULT_SETTINGS_BITS) noexcept;

    std::uint8_t getMeasurementResolution() const noexcept;
    void setMeasurementResolution(const std::uint8_t res);

    VddStatus getVddStatus() const noexcept;

    HeaterStatus getHeaterStatus() const noexcept;
    void setHeaterStatus(const HeaterStatus status) noexcept;

    void resetSettings() override;

};

struct UserRegister2 : public UserRegister {
    UserRegister2(const std::uint8_t bits = SI7021::DEFAULT_HEATER_BITS) noexcept;
    std::uint8_t getHeaterPower() const noexcept;
    void setHeaterPower(const uint8_t power);
    void resetSettings() override;
};

class SI7021 {

/**
 * Datasheet
 * https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf
 */

protected:

    int _handle;
    const int _device;
    const int _addr;

    double _temperature;
    double _humidity;

    UserRegister1 _read_user_reg_1() const;
    void _set_user_reg_1(const UserRegister1* const reg);

    UserRegister2 _read_user_reg_2() const;
    void _set_user_reg_2(const UserRegister2* const reg);

    void _i2cMultiRead(
        const std::uint8_t* const cmd,
        const std::size_t cmdLen,
        std::uint8_t* const data,
        const std::size_t dataLen) const;

    void _i2cMultiWrite(
        const std::uint8_t* const cmd,
        const std::size_t cmdLen,
        const std::uint8_t* const data = nullptr,
        const std::size_t dataLen = 0) const;

    /**
     * https://github.com/d2r2/go-si7021/blob/master/utils.go
     * https://www.silabs.com/community/sensors/forum.topic.html/how_to_calculatecrc-sCTY
     * https://circuitpython.readthedocs.io/projects/si7021/en/latest/_modules/adafruit_si7021.html
     */
    static std::uint8_t _calc_checksum(
        std::uint8_t seed,
        const std::uint8_t* bytes,
        const std::size_t byteLen) noexcept;

public:

    static const int I2C_DEV = 1;
    static const int IC2_ADDR = 0x40;

    static const std::uint8_t DEFAULT_SETTINGS_BITS = 0b00111010;
    static const std::uint8_t DEFAULT_HEATER_BITS = 0b00000000;

    static const std::uint8_t MEASURE_HUM_HOLD_MASTER = 0xE5;
    static const std::uint8_t MEASURE_HUM_NO_HOLD_MASTER = 0xF5;
    static const std::uint8_t MEASURE_TEMP_HOLD_MASTER = 0xE3;
    static const std::uint8_t MEASURE_TEMP_NO_HOLD_MASTER = 0xF3;
    static const std::uint8_t READ_TEMP_FROM_PREV_HUM_MEASURE = 0xE0;
    static const std::uint8_t RESET = 0xFE;
    static const std::uint8_t WRITE_RHT_USR_REG_1 = 0xE6;
    static const std::uint8_t READ_RHT_USR_REG_1 = 0xE7;
    static const std::uint8_t WRITE_HTR_CTRL_REG = 0x51;
    static const std::uint8_t READ_HTR_CTRL_REG = 0x11;
    static const std::uint8_t READ_ELEC_ID_1_BYTE[2];
    static const std::uint8_t READ_ELEC_ID_2_BYTE[2];
    static const std::uint8_t READ_FIRMWARE_REV[2];

    SI7021(const int dev = I2C_DEV, const int addr = IC2_ADDR) noexcept;
    virtual ~SI7021();

    void setup();
    void close();

    /**
     * Measurement takes time depending on bit resolution
     * https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf
     * pg. 5
     * Max time would be 12ms + 10.8ms = 22.8ms
     */
    void refresh();
    double getTemperature() const noexcept;
    double getHumidity() const noexcept;

    /**
     * No action should be taken for 15ms after resetting the device
     * https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf
     * pg. 5
     */
    void reset();
    
    void resetSettings();
    void resetHeater();

    std::uint8_t getMeasurementResolution() const;
    void setMeasurementResolution(const std::uint8_t res = 0);

    VddStatus getVddStatus() const;

    HeaterStatus getHeaterStatus() const;
    void setHeaterStatus(const HeaterStatus status);

    std::uint8_t getHeaterPower() const;
    void setHeaterPower(const std::uint8_t power = 0);

    SerialNumber getSerialNumber() const;
    DeviceId getDeviceId() const;
    FirmwareRevision getFirmwareRevision() const;

};
};
#endif