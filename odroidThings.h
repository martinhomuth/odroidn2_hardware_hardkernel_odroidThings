
#ifndef ANDROID_INCLUDE_HARDWARE_ODROID_THINGS_H
#define ANDROID_INCLUDE_HARDWARE_ODROID_THINGS_H

#include <hardware/hardware.h>
#include <vector>
#include <string>

#define ODROID_THINGS_HARDWARE_MODULE_ID    "odroid_things"

#define PIN_MAX 40

enum pin_mode {
    PIN_PWR = 1 << 0,
    PIN_GND = 1 << 1,
    PIN_GPIO = 1 << 2,
    PIN_AIN = 1 << 3,
    PIN_PWM = 1 << 4,
    PIN_I2C_SDA = 1 << 5,
    PIN_I2C_SCL = 1 << 6,
    PIN_SPI_SCLK = 1 << 7,
    PIN_SPI_MOSI = 1 << 8,
    PIN_SPI_MISO = 1 << 9,
    PIN_SPI_CE0 = 1 << 10,
    PIN_UART_RX = 1 << 11,
    PIN_UART_TX = 1 << 12,
    PIN_ETC = 1 << 13,
};

typedef struct pin{
    std::string name;
    int pin;
} pin_t; 

typedef struct gpio_operations {
    const std::vector<pin_t> (*getGpioList)();
    int (*getValue)();
} gpio_operations_t;

typedef struct spi_operations {
} spi_operations_t;

typedef struct things_device {
    hw_device_t common;
    gpio_operations_t gpio_ops;
    //spi_operations_t spi_ops;
} things_device_t;

typedef struct things_module {
    struct hw_module_t common;
    void (*init)();
} things_module_t;

#endif
