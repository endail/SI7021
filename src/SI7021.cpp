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

#include "../include/SI7021.h"
#include <cstring>
#include <cstdint>
#include <lgpio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <unordered_map>
#include <stdexcept>
#include <type_traits>

namespace SI7021 {

UserRegister::UserRegister() noexcept :
    UserRegister(0) { }

UserRegister::UserRegister(const std::uint8_t v) noexcept :
    std::bitset<8>(v) { }

UserRegister::UserRegister(const std::bitset<8> bs) noexcept :
    UserRegister(static_cast<std::uint8_t>(bs.to_ulong())) { }

std::uint8_t UserRegister::to_uint8_t() const noexcept {
    return static_cast<std::uint8_t>(this->to_ulong());
}

UserRegister1::UserRegister1(const std::uint8_t bits) noexcept :
    UserRegister(bits) { }

std::uint8_t UserRegister1::getMeasurementResolution() const noexcept {
    return (this->operator[](7) << 1) | this->operator[](0);
}

void UserRegister1::setMeasurementResolution(const std::uint8_t res) {

    if(res > _MAX_MEASUREMENT_RESOLUTION) {
        throw std::range_error("measurement resolution out of range");
    }

    this->operator[](7) = res & 0b00000010;
    this->operator[](0) = res & 0b00000001;

}

VddStatus UserRegister1::getVddStatus() const noexcept {
    return static_cast<VddStatus>(this->operator[](6));
}

HeaterStatus UserRegister1::getHeaterStatus() const noexcept {
    return static_cast<HeaterStatus>(this->operator[](2));
}

void UserRegister1::setHeaterStatus(const HeaterStatus status) noexcept {
    this->operator[](2) = static_cast<bool>(status);
}

void UserRegister1::resetSettings() {
    this->reset();
    this->operator[](5) = true;
    this->operator[](4) = true;
    this->operator[](3) = true;
    this->operator[](1) = true;
}

HeaterControlRegister::HeaterControlRegister(const std::uint8_t bits) noexcept :
    UserRegister(bits) { }

std::uint8_t HeaterControlRegister::getHeaterPower() const noexcept {
    return this->to_ulong() & 0b00001111;
}

void HeaterControlRegister::setHeaterPower(const uint8_t power) {

    if(power > _MAX_HEATER_POWER) {
        throw std::range_error("heater power out of range");
    }

    this->operator[](3) = power & 0b00001000;
    this->operator[](2) = power & 0b00000100;
    this->operator[](1) = power & 0b00000010;
    this->operator[](0) = power & 0b00000001;

}

void HeaterControlRegister::resetSettings() {
    this->reset();
}

const std::unordered_map<const Command, const std::uint8_t* const> SI7021::_CMD_REGS({
    { Command::MEASURE_HUM_HOLD_MASTER,         (std::uint8_t[]){ 0xe5 } },
    { Command::MEASURE_HUM_NO_HOLD_MASTER,      (std::uint8_t[]){ 0xf5 } },
    { Command::MEASURE_TEMP_HOLD_MASTER,        (std::uint8_t[]){ 0xe3 } },
    { Command::MEASURE_TEMP_NO_HOLD_MASTER,     (std::uint8_t[]){ 0xf3 } },
    { Command::READ_TEMP_FROM_PREV_HUM_MEASURE, (std::uint8_t[]){ 0xe0 } },
    { Command::RESET,                           (std::uint8_t[]){ 0xfe } },
    { Command::WRITE_RHT_USR_REG_1,             (std::uint8_t[]){ 0xe6 } },
    { Command::READ_RHT_USR_REG_1,              (std::uint8_t[]){ 0xe7 } },
    { Command::WRITE_HTR_CTRL_REG,              (std::uint8_t[]){ 0x51 } },
    { Command::READ_HTR_CTRL_REG,               (std::uint8_t[]){ 0x11 } },
    { Command::READ_ELEC_ID_1_BYTE,             (std::uint8_t[]){ 0xf1, 0x0f } },
    { Command::READ_ELEC_ID_2_BYTE,             (std::uint8_t[]){ 0xfc, 0xc9 } },
    { Command::READ_FIRMWARE_REV,               (std::uint8_t[]){ 0x84, 0xb8 } }
});

const std::unordered_map<const DeviceId, const char* const> SI7021::_DEV_STRS({
    { DeviceId::ENG_SAMPLE_1,   "engineering sample" },
    { DeviceId::ENG_SAMPLE_2,   "engineering sample" },
    { DeviceId::SI7013,         "Si7013" },
    { DeviceId::SI7020,         "Si7020" },
    { DeviceId::SI7021,         "Si7021" },
    { DeviceId::UNKNOWN,        "unknown" }
});

const std::unordered_map<const FirmwareRevision, const char* const> SI7021::_FW_STRS({
    { FirmwareRevision::REV_1_0,    "1.0" },
    { FirmwareRevision::REV_2_0,    "2.0" },
    { FirmwareRevision::UNKNOWN,    "unknown" }
});

UserRegister1 SI7021::_read_user_reg_1() const {

    std::uint8_t b = 0;

    this->_i2cMultiRead(
        _CMD_REGS.at(Command::READ_RHT_USR_REG_1),
        sizeof(_CMD_REGS.at(Command::READ_RHT_USR_REG_1)),
        &b,
        sizeof(b));

    return UserRegister1(b);

}

void SI7021::_set_user_reg_1(const UserRegister1* const reg) {

    const std::uint8_t b = reg->to_uint8_t();

    this->_i2cMultiWrite(
        _CMD_REGS.at(Command::WRITE_RHT_USR_REG_1),
        sizeof(_CMD_REGS.at(Command::WRITE_RHT_USR_REG_1)),
        &b,
        sizeof(b));

}

HeaterControlRegister SI7021::_read_user_reg_2() const {

    std::uint8_t b = 0;

    this->_i2cMultiRead(
        _CMD_REGS.at(Command::READ_HTR_CTRL_REG),
        sizeof(_CMD_REGS.at(Command::READ_HTR_CTRL_REG)),
        &b,
        sizeof(b));

    return HeaterControlRegister(b);

}

void SI7021::_set_user_reg_2(const HeaterControlRegister* const reg) {

    const std::uint8_t b = reg->to_uint8_t();

    this->_i2cMultiWrite(
        _CMD_REGS.at(Command::WRITE_HTR_CTRL_REG),
        sizeof(_CMD_REGS.at(Command::WRITE_HTR_CTRL_REG)),
        &b,
        sizeof(b));

}

void SI7021::_i2cMultiRead(
    const std::uint8_t* const cmd,
    const std::size_t cmdLen,
    std::uint8_t* const data,
    const std::size_t dataLen) const {

        ::lgI2cMsg_t segs[2];
    
        //write
        segs[0].addr = this->_addr;
        segs[0].flags = 0;
        segs[0].len = cmdLen;
        segs[0].buf = const_cast<std::uint8_t*>(cmd);

        //recv
        segs[1].addr = this->_addr;
        segs[1].flags = I2C_M_RD | I2C_M_NOSTART;
        segs[1].len = dataLen;
        segs[1].buf = data;

        const auto code = ::lgI2cSegments(
            this->_handle,
            segs,
            2);

        if(code != 2) {
            throw std::runtime_error("I2C read failed");
        }

}

void SI7021::_i2cMultiWrite(
    const std::uint8_t* const cmd,
    const std::size_t cmdLen,
    const std::uint8_t* const data,
    const std::size_t dataLen) const {

        ::lgI2cMsg_t seg;

        std::uint8_t buff[cmdLen + dataLen];
        std::memcpy(buff, cmd, cmdLen);
        std::memcpy(buff + cmdLen, data, dataLen);

        seg.addr = this->_addr;
        seg.flags = 0;
        seg.len = cmdLen + dataLen;
        seg.buf = buff;

        const auto code = ::lgI2cSegments(
            this->_handle,
            &seg,
            1);

        if(code != 1) {
            throw std::runtime_error("I2C write failed");
        }

}

std::uint8_t SI7021::_calc_checksum(
    std::uint8_t seed,
    const std::uint8_t* const bytes,
    const std::size_t byteLen) noexcept {

        for(std::size_t i = 0; i < byteLen; ++i) {

            seed ^= bytes[i];
            
            for(std::size_t j = 0; j < 8; ++j) {
                if(seed & 0x80) {
                    seed = (seed << 1) ^ 0x131;
                }
                else {
                    seed = (seed << 1);
                }
            }

        }

        return seed;

}

double SI7021::_rhCodeToHumidity(const std::uint16_t word) noexcept {
    //algo on pg. 21
    return ((125.0 * word) / 65536.0) - 6;
}

double SI7021::_tempCodeToTemperature(const std::uint16_t word) noexcept {
    //algo on pg. 22
    return ((175.72 * word) / 65536.0) - 46.85;
}

SI7021::SI7021(const int dev, const int addr) noexcept :
    _handle(-1),
    _device(dev),
    _addr(addr),
    _temperature(0.0),
    _humidity(0.0) {
}

SI7021::~SI7021() noexcept {
    this->close();
}

void SI7021::setup() {

    if(this->_handle >= 0) {
        //already setup
        return;
    }

    if((this->_handle = ::lgI2cOpen(this->_device, this->_addr, 0)) < 0) {
        throw std::runtime_error("failed to setup SI7021");
    }


}

void SI7021::close() {

    if(this->_handle == -1) {
        //already closed
        return;
    }

    if(::lgI2cClose(this->_handle) != 0) {
        throw std::runtime_error("failed to close SI7021");
    }

    this->_handle = -1;

}

void SI7021::refresh() {

    std::uint8_t data[3];

    this->_i2cMultiRead(
        _CMD_REGS.at(Command::MEASURE_HUM_HOLD_MASTER),
        std::extent<decltype(_CMD_REGS.at(Command::MEASURE_HUM_HOLD_MASTER))>::value,
        data,
        sizeof(data));

    const auto crc = this->_calc_checksum(
        0x0,
        data,
        2);

    if(crc != data[2]) {
        throw std::runtime_error("CRC checksum failed");
    }

    //read humidity from returned data
    this->_humidity = _rhCodeToHumidity(
        static_cast<std::uint16_t>(data[0]) << 8 | 
        static_cast<std::uint16_t>(data[1])
        );

    //and also grab the temp

    //clear data arr
    //note: data[2] was only used for previous crc;
    //there is no crc for this read
    std::memset(data, 0, sizeof(data));

    this->_i2cMultiRead(
        _CMD_REGS.at(Command::READ_TEMP_FROM_PREV_HUM_MEASURE),
        std::extent<decltype(_CMD_REGS.at(Command::READ_TEMP_FROM_PREV_HUM_MEASURE))>::value,
        data,
        sizeof(data));

    this->_temperature = _tempCodeToTemperature(
        static_cast<std::uint16_t>(data[0]) << 8 |
        static_cast<std::uint16_t>(data[1])
        );

}

double SI7021::getTemperature() const noexcept {
    return this->_temperature;
}

double SI7021::getHumidity() const noexcept {
    return this->_humidity;
}

void SI7021::reset() {
    this->_i2cMultiWrite(
        _CMD_REGS.at(Command::RESET),
        sizeof(_CMD_REGS.at(Command::RESET)));
}

void SI7021::resetSettings() {
    const UserRegister1 reg;
    this->_set_user_reg_1(&reg);
}

void SI7021::resetHeater() {
    const HeaterControlRegister reg;
    this->_set_user_reg_2(&reg);
}

std::uint8_t SI7021::getMeasurementResolution() const {
    return this->_read_user_reg_1().getMeasurementResolution();
}

void SI7021::setMeasurementResolution(const std::uint8_t res) {
    auto reg = this->_read_user_reg_1();
    reg.setMeasurementResolution(res);
    this->_set_user_reg_1(&reg);
}

VddStatus SI7021::getVddStatus() const {
    return this->_read_user_reg_1().getVddStatus();
}

HeaterStatus SI7021::getHeaterStatus() const {
    return this->_read_user_reg_1().getHeaterStatus();
}

std::uint8_t SI7021::getHeaterPower() const {
    return this->_read_user_reg_2().getHeaterPower();
}

void SI7021::setHeaterPower(const std::uint8_t power) {
    HeaterControlRegister reg;
    reg.setHeaterPower(power);
    this->_set_user_reg_2(&reg);
}

void SI7021::setHeaterStatus(const HeaterStatus status) {
    auto reg = this->_read_user_reg_1();
    reg.setHeaterStatus(status);
    this->_set_user_reg_1(&reg);
}

SerialNumber SI7021::getSerialNumber() const {

    //algo on pg. 23

    //first read
    std::uint8_t sna[8];

    this->_i2cMultiRead(
        _CMD_REGS.at(Command::READ_ELEC_ID_1_BYTE),
        sizeof(_CMD_REGS.at(Command::READ_ELEC_ID_1_BYTE)),
        sna,
        sizeof(sna));

    const auto crcSna3 = _calc_checksum(    0x0, &sna[0], 1);
    const auto crcSna2 = _calc_checksum(crcSna3, &sna[2], 1);
    const auto crcSna1 = _calc_checksum(crcSna2, &sna[4], 1);
    const auto crcSna0 = _calc_checksum(crcSna1, &sna[6], 1);

    if(!(
        crcSna3 == sna[1] &&
        crcSna2 == sna[3] &&
        crcSna1 == sna[5] &&
        crcSna0 == sna[7]
    )) {
        throw std::runtime_error("CRC checksum failed");
    }

    //second read
    std::uint8_t snb[6];

    this->_i2cMultiRead(
        _CMD_REGS.at(Command::READ_ELEC_ID_2_BYTE),
        sizeof(_CMD_REGS.at(Command::READ_ELEC_ID_2_BYTE)),
        snb,
        sizeof(snb));

    const auto crcSnb2 = _calc_checksum(    0x0, &snb[0], 2);
    const auto crcSnb0 = _calc_checksum(crcSnb2, &snb[3], 2);

    if(!(
        crcSnb2 == snb[2] &&
        crcSnb0 == snb[5]
    )) {
        throw std::runtime_error("CRC checksum failed");
    }

    const SerialNumber sn = 
        static_cast<std::uint64_t>(sna[0]) << 56 |
        static_cast<std::uint64_t>(sna[2]) << 48 |
        static_cast<std::uint64_t>(sna[4]) << 40 |
        static_cast<std::uint64_t>(sna[6]) << 32 |
        static_cast<std::uint64_t>(snb[0]) << 24 |
        static_cast<std::uint64_t>(snb[1]) << 16 |
        static_cast<std::uint64_t>(snb[3]) << 8  |
        static_cast<std::uint64_t>(snb[4]);

    return sn;

}

DeviceId SI7021::getDeviceId() const {

    std::uint8_t b = 0;

    this->_i2cMultiRead(
        _CMD_REGS.at(Command::READ_ELEC_ID_2_BYTE),
        sizeof(_CMD_REGS.at(Command::READ_ELEC_ID_2_BYTE)),
        &b,
        sizeof(b));

    const auto id = static_cast<DeviceId>(b);

    switch(id) {
        case DeviceId::ENG_SAMPLE_1:
        case DeviceId::ENG_SAMPLE_2:
        case DeviceId::SI7013:
        case DeviceId::SI7020:
        case DeviceId::SI7021:
            return id;
        default:
            return DeviceId::UNKNOWN;
    }

}

FirmwareRevision SI7021::getFirmwareRevision() const {

    std::uint8_t b = 0;

    this->_i2cMultiRead(
        _CMD_REGS.at(Command::READ_FIRMWARE_REV),
        sizeof(_CMD_REGS.at(Command::READ_FIRMWARE_REV)),
        &b,
        sizeof(b));

    const auto fw = static_cast<FirmwareRevision>(b);

    switch(fw) {
        case FirmwareRevision::REV_1_0:
        case FirmwareRevision::REV_2_0:
            return fw;
        default:
            return FirmwareRevision::UNKNOWN;
    }

}

const char* const SI7021::devIdToString(const DeviceId id) noexcept {
    return _DEV_STRS.at(id);
}

const char* const SI7021::firmwareRevToString(const FirmwareRevision rev) noexcept {
    return _FW_STRS.at(rev);
}

};