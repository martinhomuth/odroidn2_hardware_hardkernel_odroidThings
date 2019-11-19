/*
 *    Copyright (c) 2019 Sangchul Go <luke.go@hardkernel.com>
 *
 *    OdroidThings is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    OdroidThings is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with OdroidThings.
 *    If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#include <hardware/odroidThings.h>
#include <wiringPi/wiringPi.h>
#include <vector>
#include <map>
#include <fstream>
#include <string>

#define BOARD_PROPERTY "ro.product.device"

using hardware::hardkernel::odroidthings::pin_t;
using hardware::hardkernel::odroidthings::i2c_t;
using hardware::hardkernel::odroidthings::function_t;
using std::string;

struct pwmState{
    unsigned int period;
    double cycle_rate;

    uint8_t chip;
    uint8_t node;
    std::string periodPath;
    std::string dutyCyclePath;
    std::string enablePath;
    std::string unexportPath;
};

class PinManager {
    private:
        string board;
        pin_t *pinList;
        i2c_t *i2cList;
        int triggerType[PIN_MAX] = {INT_EDGE_SETUP,};
        std::map<int, pwmState *> pwm;
        std::map<int, int> i2c;

        void initPwm();

        // helper function
        void initPwmState(int idx, uint8_t chip, uint8_t node);
        void writeSysfsTo(const string path, const string value);

        enum ActiveType {
            ACTIVE_LOW,
            ACTIVE_HIGH,
        };
        enum EdgeTrigger {
            EDGE_NONE,
            EDGE_RISING,
            EDGE_FALLING,
            EDGE_BOTH,
        };
    public:
        PinManager();
        void init();
        std::vector<pin_t> getPinList();

        // common
        std::vector<string> getPinNameList();
        std::vector<string> getListOf(int);

        // gpio
        bool getValue(int);
        void setDirection(int, direction_t);
        void setValue(int, bool);
        void setActiveType(int, int);
        void setEdgeTriggerType(int, int);
        void registerCallback(int, function_t);
        void unregisterCallback(int);

        // pwm
        void openPwm(int);
        void closePwm(int);
        bool setPwmEnable(int, bool);
        bool setPwmDutyCycle(int, double);
        bool setPwmFrequency(int, double);

        // i2c
        void openI2c(int, uint32_t, int);
        void closeI2c(int);
        std::vector<uint8_t> readRegBufferI2c(int, uint32_t, int);
        Result writeRegBufferI2c(int, uint32_t, std::vector<uint8_t>, int);
};

#endif /* PIN_MANAGER_H */
