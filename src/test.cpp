// MIT License
//
// Copyright (c) 2021 Daniel Robertson
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

#include <chrono>
#include <iostream>
#include <thread>
#include "../include/SI7021.h"

int main() {

    using namespace std;
    using namespace SI7021;

    SI7021::SI7021 sensor;

    sensor.setup();

    std::cout

        << "Device: "
        << SI7021::SI7021::devIdToString(sensor.getDeviceId()) << "\n"

        << "Firmware: "
        << SI7021::SI7021::firmwareRevToString(sensor.getFirmwareRevision()) << "\n"
        
        << "Serial Number: "
        << sensor.getSerialNumber() << "\n"
        
        << "Vdd Status: "
        << (sensor.getVddStatus() == VddStatus::OK ? "OK" : "Low") << "\n"
        
        << "Heater Status: "
        << (sensor.getHeaterStatus() == HeaterStatus::ENABLE ? "Enabled" : "Disabled") << "\n"

        << "Heater Power: "
        << static_cast<int>(sensor.getHeaterPower()) << "\n"

        << "Measurement Resolution: "
        << static_cast<int>(sensor.getMeasurementResolution()) << "\n"

        << std::endl;

    while(true) {

        std::this_thread::sleep_for(std::chrono::seconds(1));

        sensor.refresh();

        std::cout 
            << "Temperature: " << sensor.getTemperature() << " C, "
            << "Humidity: " << sensor.getHumidity() << " %RH"
            << "\n";

    }

    return 0;

}
