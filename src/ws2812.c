#include "ws2812.h"
#include <drivers/led_strip.h>
#include <drivers/spi.h>
#include <string.h>

/* size of stack area used by each thread */
#define STACKSIZE 512

/* scheduling priority used by each thread */
#define PRIORITY 7

#define TASK_SLEEP(x)                                     k_sleep(K_MSEC(x))

#define STRIP_LABEL                                       DT_LABEL(DT_ALIAS(led_strip))
#define STRIP_NUM_PIXELS                                  DT_PROP(DT_ALIAS(led_strip), chain_length)

#define RGB(_r, _g, _b)   { .r = (_r), .g = (_g), .b = (_b) }

const struct device* strip;

static const struct led_rgb colors[] = {
    RGB(0x0f, 0x00, 0x00), /* red */
    RGB(0x00, 0x0f, 0x00), /* green */
    RGB(0x00, 0x00, 0x0f), /* blue */
};

struct led_rgb pixels[STRIP_NUM_PIXELS];



static void ws2812_init(void)
{
    strip = device_get_binding(STRIP_LABEL);
    if (strip) {
        printk("Found LED strip device %s - OK\n", STRIP_LABEL);
    }
    else {
        printk("LED strip device %s not found\n", STRIP_LABEL);
        return;
    }
}

void ws2812_task(void)
{
    ws2812_init();

    size_t cursor = 0, color = 0;
    int rc;
    printk("Start displaying pattern on strip\n");
    uint8_t battery = 0;

    while (1) {
        battery++;
        if (battery > 100)
            battery = 0;

        memset(&pixels, 0x00, sizeof(pixels));
        memcpy(&pixels[cursor], &colors[color], sizeof(struct led_rgb));
        rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);

        if (rc) {
            printk("couldn't update strip: %d\n", rc);
        }

        cursor++;
        if (cursor >= STRIP_NUM_PIXELS) {
            cursor = 0;
            color++;
            if (color == ARRAY_SIZE(colors)) {
                color = 0;
            }
        }

        TASK_SLEEP(500);
    }
    
}


K_THREAD_DEFINE(led_strip, STACKSIZE, ws2812_task, NULL, NULL, NULL,
		PRIORITY, 0, 0);