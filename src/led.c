#include "led.h"
#include <device.h>
#include <drivers/gpio.h>

#define LED0_NODE                                         DT_ALIAS(led0)

#define LED0                                              DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN                                               DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS                                             DT_GPIO_FLAGS(LED0_NODE, gpios)

const struct device* dev;



void led_init(void)
{
    dev = device_get_binding(LED0);
    if (dev == NULL) {
        return;
    }

    int err = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
    if (err < 0) {
        return;
    }
    gpio_pin_set(dev, PIN, 0);
}