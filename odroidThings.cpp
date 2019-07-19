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

#include <hardware/hardware.h>
#include <hardware/odroidThings.h>

#include <cutils/log.h>
#include <utils/Mutex.h>

#include <string>
#include <vector>
#include <memory>
#include "PinManager.h"

static android::Mutex thingsLock;
static std::unique_ptr<PinManager> gPinManager;

static void things_init() {
    gPinManager = std::make_unique<PinManager>();
    gPinManager->init();
}

static const std::vector<pin_t> things_getPinList() {
    return gPinManager->getPinList();
}

static bool things_getValue(int pin) {
    return gPinManager->getValue(pin);
}

static void things_setDirection(int pin, int direction) {
    return gPinManager->setDirection(pin, direction);
}

static void things_setValue(int pin, bool value) {
    return gPinManager->setValue(pin, value);
}

static void things_setActiveType(int pin, int activeType) {
    gPinManager->setActiveType(pin, activeType);
}

static void things_setEdgeTriggerType(int pin, int edgeTriggerType) {
    gPinManager->setEdgeTriggerType(pin, edgeTriggerType);
}

static void things_registerCallback(int pin, function_t callback) {
    gPinManager->registerCallback(pin, callback);
}

static void things_unregisterCallback(int pin) {
    gPinManager->unregisterCallback(pin);
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

    ALOGD("things HAL open");
    if (!dev) {
        ALOGE("no memory for the odroid things device");
        return -ENOMEM;
    }

    memset(dev, 0, sizeof(*dev));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = const_cast<hw_module_t*>(module);
    dev->common.close = things_close;

    dev->gpio_ops.getValue = things_getValue;
    dev->gpio_ops.setDirection = things_setDirection;
    dev->gpio_ops.setValue = things_setValue;
    dev->gpio_ops.setActiveType = things_setActiveType;
    dev->gpio_ops.setEdgeTriggerType = things_setEdgeTriggerType;
    dev->gpio_ops.registerCallback = things_registerCallback;
    dev->gpio_ops.unregisterCallback = things_unregisterCallback;
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
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = ODROID_THINGS_HARDWARE_MODULE_ID,
        .name = " Odroid things module",
        .author = "Hardkernel",
        .methods = &things_module_methods,
        .reserved ={0},
    },
    .init = things_init,
    .getPinList = things_getPinList,
};
