#include <hardware/hardware.h>
#include <odroidThings.h>
#include <wiringPi/wiringPi.h>

#include <cutils/log.h>
#include <utils/Mutex.h>
#include <cutils/properties.h>

#include <errno.h>
#include <string>
#include <memory>

#define BOARD_PROPERTY "ro.product.device"

static android::Mutex thingsLock;

// pin array number is based on physical pin number
static const pin_t n2_pin_support_list[PIN_MAX] = {
    {"3.3V", -1}, {"5V", -1},
    {"P03", 8}, {"5V", -1},
    {"P05", 9}, {"GND", -1},
    {"P06", 7}, {"P08", 15},
    {"GND", -1}, {"P10", 16},
    {"P11", 0}, {"P12", 1},
    {"P13", 2}, {"GND", -1},
    {"P15", 3}, {"P16", 4},
    {"3.3V", -1}, {"P18", 5},
    {"P19", 12}, {"GND", -1},
    {"P21", 13}, {"P22", 6},
    {"P23", 14}, {"P24", 10},
    {"P25", -1}, {"P26", 11},
    {"P27", 30}, {"P28", 31},
    {"P29", 21}, {"GND", -1},
    {"P31", 22}, {"P32", 26},
    {"P33", 23}, {"GND", -1},
    {"P35", 24}, {"P36", 27},
    {"AIN0", 25}, {"1.8V", 28},
    {"GND", -1}, {"AIN1", 29},
};

static pin_mode n2_current_mode[PIN_MAX] {
    PIN_PWR, PIN_PWR,
    PIN_I2C_SDA, PIN_PWR,
    PIN_I2C_SCL, PIN_GND,
    PIN_GPIO, PIN_UART_TX,
    PIN_GND, PIN_UART_RX,
    PIN_GPIO, PIN_GPIO,
    PIN_GPIO, PIN_GND,
    PIN_GPIO, PIN_GPIO,
    PIN_PWR, PIN_GPIO,
    PIN_GPIO, PIN_GND,
    PIN_GPIO, PIN_GPIO,
    PIN_GPIO, PIN_GPIO,
    PIN_GND, PIN_GPIO,
    PIN_I2C_SDA, PIN_I2C_SCL,
    PIN_GPIO, PIN_GND,
    PIN_GPIO, PIN_GPIO,
    PIN_ETC, PIN_GND,
    PIN_ETC, PIN_GPIO,
    PIN_AIN, PIN_PWR,
    PIN_GND, PIN_AIN
};

class PinManager {
    private:
        std::string board;
        pin_t *pinList;
        pin_mode *currentMode;

    public:
        PinManager();
        void init();
        std::vector<pin_t> getGpioList();
        int getValue();
};

static std::unique_ptr<PinManager> gPinManager;

PinManager::PinManager(){
    char boardName[PROPERTY_VALUE_MAX];

    property_get(BOARD_PROPERTY, boardName, NULL);
    board = boardName;
}

void PinManager::init() {
    if (board == "odroidn2") {
        pinList = (pin_t*)n2_pin_support_list;
        currentMode = n2_current_mode;
    }
}

std::vector<pin_t> PinManager::getGpioList() {
    std::vector<pin_t> gpioList;

    for (int i = 0; i < PIN_MAX; i++) {
        if (currentMode[i] == PIN_GPIO) {
            gpioList.push_back(pinList[i]);
        }
    }
    return gpioList;
}

int PinManager::getValue() {
    return 0;
}

static void things_init() {
    wiringPiSetupGpio();

    gPinManager = std::make_unique<PinManager>();
    gPinManager->init();
}

static const std::vector<pin_t> things_getGpioList() {
    return gPinManager->getGpioList();
}

static int things_getValue() {
    return gPinManager->getValue();
}

static int things_close(struct hw_device_t *dev) {
    android::Mutex::Autolock lock(thingsLock);

    things_device_t *device = (things_device_t*)dev;
    if (device) {
        free(device);
        device = NULL;
    }
    return 0;
}

static int things_open(const hw_module_t *module, const char __unused *id,
        struct hw_device_t **device) {
    android::Mutex::Autolock lock(thingsLock);

    things_device_t *dev = (things_device_t*)malloc(sizeof(things_device_t));

    if (!dev) {
        ALOGE("no memory for the odroid things device");
        return -ENOMEM;
    }

    memset(dev, 0, sizeof(*dev));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = const_cast<hw_module_t*>(module);
    dev->common.close = things_close;

    dev->gpio_ops.getGpioList = things_getGpioList;
    dev->gpio_ops.getValue = things_getValue;
    //dev->spi_ops.

    *device = &dev->common;

    return 0;
}

static struct hw_module_methods_t things_module_methods = {
    .open = things_open,
};

things_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .version_major = 1,
        .version_minor = 0,
        .id = ODROID_THINGS_HARDWARE_MODULE_ID,
        .name = " Odroid things module",
        .author = "Hardkernel",
        .methods = &things_module_methods,
        .dso = NULL,
        .reserved ={0},
    },
    .init = things_init,
};
