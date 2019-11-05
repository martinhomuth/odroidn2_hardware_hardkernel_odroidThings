#include <wiringPi/wiringPi.h>
#include <cutils/properties.h>
#include <hardware/odroidThings.h>
#include <hardware/odroidthings-base.h>

#include <string>
#include <sstream>
#include <vector>
#include <map>

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
    initPwm();
}

std::vector<pin_t> PinManager::getPinList() {
    std::vector<pin_t> list;

    for (int i=0; i<PIN_MAX; i++)
        list.push_back(pinList[i]);
    return list;
}

std::vector<std::string> PinManager::getPinNameList() {
    std::vector<std::string> list;

    for (int i=0; i<PIN_MAX; i++)
        list.push_back(pinList[i].name);
    return list;
}

std::vector<std::string> PinManager::getListOf(int mode) {
    std::vector<std::string> list;
    for (int i=0; i<PIN_MAX; i++) {
        switch (mode) {
            case PIN_GPIO:
                if (pinList[i].availableModes & PIN_GPIO) {
                    int alt = getAlt(pinList[i].pin);
                    if (alt < 2)
                        list.push_back(pinList[i].name);
                }
                break;
            case PIN_PWM:
                if (pinList[i].availableModes & PIN_PWM) {
                    int alt = getAlt(pinList[i].pin);
                    ALOGD("Board alt num is -  %d", alt);

                    if (alt < 2) {
                        list.push_back(pinList[i].name);
                    }
                    if ((pinList[i].pin < 4) && (alt == 2))
                        list.push_back(pinList[i].name);
                    else if ((pinList[i].pin > 22) && (alt == 5))
                        list.push_back(pinList[i].name);
                }
                break;
        }
    }
    return list;
}

bool PinManager::getValue(int idx) {
    return (digitalRead(pinList[idx].pin) == HIGH);
}

void PinManager::setDirection(int idx, direction_t direction) {
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

#define PWM_RANGE_MAX 65535

//TODO: reduce pwm array size to fit the pwm number.

void PinManager::initPwm() {
    for (int i=0; i<PIN_MAX; i++) {
        if ((pinList[i].availableModes & PIN_PWM) == PIN_PWM)
            pwm.insert(std::make_pair(pinList[i].pin, new pwmState()));
    }

    if (board == "odroidn2") {
        // #12
        initPwmState(1, 8, 0);
        // #15
        initPwmState(3, 8, 1);
        // #33
        initPwmState(23, 4, 0);
        // #35
        initPwmState(24, 4, 1);
    }
}

void PinManager::initPwmState(int idx, uint8_t chip, uint8_t node) {
    const auto state = pwm.find(idx)->second;
    // init chip & node info
    state->chip = chip;
    state->node = node;

    // init configuration sysfs path.
    std::ostringstream pwmRoot;

    pwmRoot << "/sys/class/pwm/pwmchip" << std::to_string(state->chip)
        << "/pwm" << std::to_string(state->node) <<"/";
    std::string pwmRootStr = pwmRoot.str();

    state->periodPath =  pwmRootStr + "period";
    state->dutyCyclePath = pwmRootStr + "duty_cycle";
    state->enablePath = pwmRootStr + "enable";
}

inline void writeSysfsTo(std::string path, std::string value) {
    std::ofstream file(path);
    path << value;
    path.close();
}

void PinManager::openPwm(int idx) {
    auto pin = pinList[idx].pin;
    const auto state = pwm.find(pin)->second;
    std::ostringstream openPath;

    ALOGD("openPwm(chip:%d-node:%d)", state->chip, state->node);

    openPath << "/sys/class/pwm/pwmchip" << std::to_string(state->chip) << "/";

    writeSysfsTo(openPath.str() + "export", to_string(state->node));

    state->unexportPath = openPath.str() + "unexport";

    openPath <<"pwm" << std::to_string(state->node) <<"/";
    writeSysfsTo(openPath.str() + "polarity", "normal");
}

void PinManager::closePwm(int idx) {
    auto pin = pinList[idx].pin;
    const auto state = pwm.find(pin)->second;
    state->period = 0;
    state->cycle_rate = 0;

    ALOGD("closePwm(chip:%d-node:%d)", state->chip, state->node);

    writeSysfsTo(state->enablePath, "0");
    writeSysfsTo(state->dutyCyclePath, "0");
    writeSysfsTo(state->periodPath, "0");
    writeSysfsTo(state->unexportPath, std::to_string(state->node));
}

bool PinManager::setPwmEnable(int idx, bool enabled) {
    auto pin = pinList[idx].pin;
    const auto state = pwm.find(pin)->second;

    writeSysfsTo(state->enablePath, (enable?"1":"0"));
    return true;
}

bool PinManager::setPwmDutyCycle(int idx, double cycle_rate) {
    auto pin = pinList[idx].pin;
    const auto state = pwm.find(pin)->second;
    unsigned int duty_cycle = (state->period / 100) * cycle_rate;

    writeSysfsTo(state->dutyCyclePath, std::to_string(duty_cycle));

    state->cycle_rate = cycle_rate;

    return true;
}

#define HERTZTONANOSECOND 1000000000
bool PinManager::setPwmFrequency(int idx, double frequency_hz) {
    auto pin = pinList[idx].pin;
    const auto state = pwm.find(pin)->second;

    state->period = HERTZTONANOSECOND / frequency_hz;

    writeSysfsTo(state->periodPath, std::to_string(state->period));

    if (state->cycle_rate != 0) {
        setPwmDutyCycle(idx, state->cycle_rate);
    }

    return true;
}
