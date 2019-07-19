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

#define BOARD_PROPERTY "ro.product.device"

class PinManager {
    private:
        std::string board;
        pin_t *pinList;
        int triggerType[PIN_MAX] = {INT_EDGE_SETUP,};

        enum Direction {
            DIRECTION_IN,
            DIRECTION_OUT_INITIALLY_HIGH,
            DIRECTION_OUT_INITIALLY_LOW,
        };
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
        bool getValue(int);
        void setDirection(int, int);
        void setValue(int, bool);
        void setActiveType(int, int);
        void setEdgeTriggerType(int, int);
        void registerCallback(int, function_t);
        void unregisterCallback(int);
};

#endif /* PIN_MANAGER_H */
