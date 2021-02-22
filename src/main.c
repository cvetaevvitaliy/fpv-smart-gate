/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <errno.h>
#include <device.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <settings/settings.h>

#include "bluetooth.h"
#include "bluetooth_mesh.h"
#include "ws2812.h"
#include "led.h"

#include <debug/cpu_load.h>

#include <shell/shell.h>
#include <version.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"


static int cmd_version(const struct shell* shell, size_t argc, char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(shell, "Zephyr version %s", KERNEL_VERSION_STRING);

    return 0;
}

SHELL_CMD_ARG_REGISTER(version, NULL, "Show kernel version", cmd_version, 1, 0);

void main(void) {

    k_sleep(K_MSEC(100));
    printk("\nBuild time: %s %s\n", __DATE__, __TIME__);
    printk("Initializing...\n");

    cpu_load_init();

    led_init();

    bluetooth_init();

    bt_bas_set_battery_level(55);
}

