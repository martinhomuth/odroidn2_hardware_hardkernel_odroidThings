#include <wiringPi/wiringPi.h>
#include <cutils/properties.h>
#include <hardware/odroidThings.h>
#include <string>
#include <vector>
#include <cutils/log.h>

#include "PinManager.h"

// pin array number is based on physical pin number
static const pin_t n2_pin_support_list[PIN_MAX] = {
    {"3.3V", -1, PIN_PWR}, {"5V", -1, PIN_PWR},
    {"3", 8, PIN_I2C_SDA}, {"5V", -1, PIN_PWR},
    {"5", 9, PIN_I2C_SCL}, {"GND", -1, PIN_GND},
    {"7", 7, PIN_GPIO}, {"8", 15, PIN_UART_TX},
    {"GND", -1, PIN_GND}, {"10", 16, PIN_UART_RX},
    {"11", 0, PIN_GPIO}, {"12", 1, PIN_GPIO|PIN_PWM},
    {"13", 2, PIN_GPIO}, {"GND", -1, PIN_GND},
    {"15", 3, PIN_GPIO|PIN_PWM}, {"16", 4, PIN_GPIO},
    {"3.3V", -1, PIN_PWR}, {"18", 5, PIN_GPIO},
    {"19", 12, PIN_GPIO}, {"GND", -1, PIN_GND},
    {"21", 13, PIN_GPIO}, {"22", 6, PIN_GPIO},
    {"23", 14, PIN_GPIO}, {"24", 10, PIN_GPIO},
    {"25", -1, PIN_GND}, {"26", 11, PIN_GPIO},
    {"27", 30, PIN_I2C_SDA}, {"28", 31, PIN_I2C_SCL},
    {"29", 21, PIN_GPIO}, {"GND", -1, PIN_GND},
    {"31", 22, PIN_GPIO}, {"32", 26, PIN_GPIO},
    {"33", 23, PIN_GPIO|PIN_PWM}, {"GND", -1, PIN_GND},
    {"35", 24, PIN_GPIO|PIN_PWM}, {"36", 27, PIN_GPIO},
    {"AIN0", 25, PIN_AIN}, {"1.8V", 28, PIN_PWR},
    {"GND", -1, PIN_GND}, {"AIN1", 29, PIN_AIN},
};

PinManager::PinManager(){
    char boardName[PROPERTY_VALUE_MAX];

    property_get(BOARD_PROPERTY, boardName, NULL);
    board = boardName;

    ALOGD("Board is %s", board.c_str());
}

void PinManager::init() {
    wiringPiSetup();

    if (board == "odroidn2") {
        pinList = (pin_t*)n2_pin_support_list;
    }
}

std::vector<pin_t> PinManager::getPinList() {
    std::vector<pin_t> list;

    for (int i=0; i<PIN_MAX; i++)
        list.push_back(pinList[i]);
    return list;
}

bool PinManager::getValue(int idx) {
    return (digitalRead(pinList[idx].pin) == HIGH);
}

void PinManager::setDirection(int idx, int direction) {
    int pin = pinList[idx].pin;
    switch (direction) {
        case DIRECTION_IN:
            pinMode(pin, INPUT);
            break;
        case DIRECTION_OUT_INITIALLY_HIGH:
            pinMode(pin, OUTPUT);
            digitalWrite(pin, HIGH);
            break;
        case DIRECTION_OUT_INITIALLY_LOW:
            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
            break;
    }
}

void PinManager::setValue(int idx, bool value) {
    digitalWrite(pinList[idx].pin, value?HIGH:LOW);
}

void PinManager::setActiveType(int idx, int activeType) {
    int pin = pinList[idx].pin;
    switch (activeType) {
        case ACTIVE_LOW:
            pullUpDnControl(pin, PUD_UP);
            break;
        case ACTIVE_HIGH:
            pullUpDnControl(pin, PUD_DOWN);
            break;
    }
}

void PinManager::setEdgeTriggerType(int idx, int edgeTriggerType) {
    int pin = pinList[idx].pin;
    switch (edgeTriggerType) {
        case EDGE_NONE:
            return;
            break;
        case EDGE_RISING:
            triggerType[pin] = INT_EDGE_RISING;
            break;
        case EDGE_FALLING:
            triggerType[pin] = INT_EDGE_FALLING;
            break;
        case EDGE_BOTH:
            triggerType[pin] = INT_EDGE_BOTH;
    }
}

void PinManager::registerCallback(int idx, function_t callback) {
    int pin = pinList[idx].pin;
    wiringPiISR(pin, triggerType[pin], callback);
}

void PinManager::unregisterCallback(int idx) {
    wiringPiISRCancel(pinList[idx].pin);
}
