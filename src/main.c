/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/printk.h>

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>
#include <bluetooth/services/bas.h>
#include <device.h>
#include <drivers/gpio.h>

#include "board.h"
#include <debug/cpu_load.h>

#include <logging/log.h>
#include <shell/shell.h>
#include <stdlib.h>
#include <version.h>

#include <device.h>
#include <drivers/led_strip.h>
#include <drivers/spi.h>
#include <sys/util.h>
#include <zephyr.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "common.h"

 /* Model Operation Codes */
#define BT_MESH_MODEL_OP_GEN_ONOFF_GET                    BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET                    BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK              BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GEN_ONOFF_STATUS                 BT_MESH_MODEL_OP_2(0x82, 0x04)

#define LED0_NODE                                         DT_ALIAS(led0)
const struct device* dev;

#define LED0                                              DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN                                               DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS                                             DT_GPIO_FLAGS(LED0_NODE, gpios)

#define STRIP_LABEL                                       DT_LABEL(DT_ALIAS(led_strip))
#define STRIP_NUM_PIXELS                                  8 // DT_PROP(DT_ALIAS(led_strip), chain_length)

#define DELAY_TIME                                        K_MSEC(50)

#define RGB(_r, _g, _b)   { .r = (_r), .g = (_g), .b = (_b) }

static const struct led_rgb colors[] = {
    RGB(0x0f, 0x00, 0x00), /* red */
    RGB(0x00, 0x0f, 0x00), /* green */
    RGB(0x00, 0x00, 0x0f), /* blue */
};

struct led_rgb pixels[STRIP_NUM_PIXELS];

static void gen_onoff_set(struct bt_mesh_model* model,
    struct bt_mesh_msg_ctx* ctx,
    struct net_buf_simple* buf);

static void gen_onoff_set_unack(struct bt_mesh_model* model,
    struct bt_mesh_msg_ctx* ctx,
    struct net_buf_simple* buf);

static void gen_onoff_get(struct bt_mesh_model* model,
    struct bt_mesh_msg_ctx* ctx,
    struct net_buf_simple* buf);

static void gen_onoff_status(struct bt_mesh_model* model,
    struct bt_mesh_msg_ctx* ctx,
    struct net_buf_simple* buf);

static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
    {BT_MESH_MODEL_OP_GEN_ONOFF_STATUS, 1, gen_onoff_status},
    BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_cfg_srv cfg_srv = {
    .relay = BT_MESH_RELAY_ENABLED,
    .beacon = BT_MESH_BEACON_ENABLED,
    .frnd = BT_MESH_FRIEND_ENABLED,
    .gatt_proxy = BT_MESH_GATT_PROXY_ENABLED,
    .default_ttl = 7,

    /* 3 transmissions with 20ms interval */
    .net_transmit = BT_MESH_TRANSMIT(2, 20),
    .relay_retransmit = BT_MESH_TRANSMIT(2, 20),
};

typedef struct onoff_state {
    uint8_t current;
    uint8_t led_gpio_pin;
    const struct device* led_device;
} onoff_state_t;
onoff_state_t onoff_state = { 0 };

static void attention_on(struct bt_mesh_model* model) {}

static void attention_off(struct bt_mesh_model* model) {}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
    .attn_on = attention_on,
    .attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

static struct bt_mesh_model_pub gen_level_pub;
static struct bt_mesh_model_pub gen_onoff_pub;

static void gen_onoff_get(struct bt_mesh_model* model,
    struct bt_mesh_msg_ctx* ctx,
    struct net_buf_simple* buf) {
    printk("gen_onoff_get()\n");

    NET_BUF_SIMPLE_DEFINE(msg, 2 + 1 + 4);

    printk("addr 0x%04x onoff 0x%02x\n", bt_mesh_model_elem(model)->addr,
        onoff_state.current);

    bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
    printk("pin_st: %d\n", onoff_state.current);
    net_buf_simple_add_u8(&msg, onoff_state.current);

    if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
        printk("Unable to send On Off Status response\n");
    }
}

static void gen_onoff_set(struct bt_mesh_model* model,
    struct bt_mesh_msg_ctx* ctx,
    struct net_buf_simple* buf) {

    onoff_state.current = buf->data[0];
    printk("\ngen_onoff_set() len %d, size%d, data: 0X%02X\n\n", buf->len,
        buf->size, onoff_state.current);

    gpio_pin_set(dev, PIN, onoff_state.current);

    gen_onoff_set_unack(model, ctx, buf);
    // gen_onoff_get(model, ctx, buf);
}

static void gen_onoff_set_unack(struct bt_mesh_model* model,
    struct bt_mesh_msg_ctx* ctx,
    struct net_buf_simple* buf) {

    onoff_state.current = net_buf_simple_pull_u8(buf);
    printk("addr 0x%02x state 0x%02x\n", bt_mesh_model_elem(model)->addr,
        onoff_state.current);
}

static void gen_onoff_status(struct bt_mesh_model* model,
    struct bt_mesh_msg_ctx* ctx,
    struct net_buf_simple* buf) {
    uint8_t state;

    state = net_buf_simple_pull_u8(buf);

    printk("Node 0x%04x OnOff status from 0x%04x with state 0x%02x\n",
        bt_mesh_model_elem(model)->addr, ctx->addr, state);
}

static const struct bt_mesh_model_op gen_onoff_op[] = {
    {BT_MESH_MODEL_OP_2(0x82, 0x01), 0, gen_onoff_get},
    {BT_MESH_MODEL_OP_2(0x82, 0x02), 2, gen_onoff_set},
    {BT_MESH_MODEL_OP_2(0x82, 0x03), 2, gen_onoff_set_unack},
    BT_MESH_MODEL_OP_END,
};

static void gen_level_get(struct bt_mesh_model* model,
    struct bt_mesh_msg_ctx* ctx,
    struct net_buf_simple* buf) {
    printk("\n\ngen_level_get()\n\n");
    printk("\n\n");
    for (int i = 0; i < buf->len; ++i)
        printk("data: %02X ", buf->data[i]);
    printk("\n\n");
}

static void gen_level_set(struct bt_mesh_model* model,
    struct bt_mesh_msg_ctx* ctx,
    struct net_buf_simple* buf) {
    printk("\n\ngen_level_set()\n\n");
    printk("\n\n");
    for (int i = 0; i < buf->len; ++i)
        printk("data: %02X ", buf->data[i]);
    printk("\n\n");

    int16_t level = 0;
    memcpy(&level, buf->data, 2);
    printk("level %d\n\n", level);
}

static void gen_level_set_unack(struct bt_mesh_model* model,
    struct bt_mesh_msg_ctx* ctx,
    struct net_buf_simple* buf) {}

static void gen_delta_set(struct bt_mesh_model* model,
    struct bt_mesh_msg_ctx* ctx,
    struct net_buf_simple* buf) {}

static void gen_delta_set_unack(struct bt_mesh_model* model,
    struct bt_mesh_msg_ctx* ctx,
    struct net_buf_simple* buf) {}

static void gen_move_set(struct bt_mesh_model* model,
    struct bt_mesh_msg_ctx* ctx,
    struct net_buf_simple* buf) {}

static void gen_move_set_unack(struct bt_mesh_model* model,
    struct bt_mesh_msg_ctx* ctx,
    struct net_buf_simple* buf) {}

static const struct bt_mesh_model_op gen_level_op[] = {
    {BT_MESH_MODEL_OP_2(0x82, 0x05), 0, gen_level_get},
    {BT_MESH_MODEL_OP_2(0x82, 0x06), 3, gen_level_set},
    {BT_MESH_MODEL_OP_2(0x82, 0x07), 3, gen_level_set_unack},
    {BT_MESH_MODEL_OP_2(0x82, 0x09), 5, gen_delta_set},
    {BT_MESH_MODEL_OP_2(0x82, 0x0a), 5, gen_delta_set_unack},
    {BT_MESH_MODEL_OP_2(0x82, 0x0b), 3, gen_move_set},
    {BT_MESH_MODEL_OP_2(0x82, 0x0c), 3, gen_move_set_unack},
    BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_model root_models[] = {
    BT_MESH_MODEL_CFG_SRV(&cfg_srv),
    BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_op, &gen_onoff_pub,
                  NULL),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_LEVEL_SRV, gen_level_op, &gen_level_pub,
                  NULL),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op,
                  &gen_onoff_pub, NULL),
};

static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
    .cid = BT_COMP_ID_LF,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

static int output_number(bt_mesh_output_action_t action, uint32_t number) {
    printk("OOB Number: %u\n", number);

    board_output_number(action, number);

    return 0;
}

static void prov_complete(uint16_t net_idx, uint16_t addr) {
    board_prov_complete();
}

static void prov_reset(void) {
    bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

static const uint8_t dev_uuid[16] = { 0xdd, 0xdd };

static const struct bt_mesh_prov prov = {
    .uuid = dev_uuid,
    .output_size = 4,
    .output_actions = BT_MESH_DISPLAY_NUMBER,
    .output_number = output_number,
    .complete = prov_complete,
    .reset = prov_reset,
};

static void bt_ready(int err) {
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized - Ok\n");

    board_init();

    err = bt_mesh_init(&prov, &comp);
    if (err) {
        printk("Initializing mesh failed (err %d)\n", err);
        return;
    }

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    /* This will be a no-op if settings_load() loaded provisioning info */
    bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

    printk("Mesh initialized - Ok\n");
}

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
    int err;
    cpu_load_init();

    dev = device_get_binding(LED0);
    if (dev == NULL) {
        return;
    }

    err = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
    if (err < 0) {
        return;
    }
    gpio_pin_set(dev, PIN, 0);

    printk("Initializing...\n");
    register_mgmt();

    bt_addr_le_t bt_addr;
    bt_id_get(&bt_addr, (uint32_t*)sizeof(bt_addr.a.val));
    printk("ID: %02X %02X %02X %02X %02X %02X\n", bt_addr.a.val[0],
        bt_addr.a.val[1], bt_addr.a.val[2], bt_addr.a.val[3], bt_addr.a.val[4],
        bt_addr.a.val[5]);

    char NAME_NODE[20];
    sprintf(NAME_NODE, "Smart Gate %02X%02X", bt_addr.a.val[4], bt_addr.a.val[5]);
    bt_set_name((const char*)NAME_NODE);

    printk("BT name: %s\n", bt_get_name());

    /* Initialize the Bluetooth Subsystem */
    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
    }

    start_smp_bluetooth();

    const struct device* strip;
    size_t cursor = 0, color = 0;
    int rc;

    strip = device_get_binding(STRIP_LABEL);
    if (strip) {
        printk("Found LED strip device %s - OK\n", STRIP_LABEL);
    }
    else {
        printk("LED strip device %s not found\n", STRIP_LABEL);
        return;
    }

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

        bt_bas_set_battery_level(battery);

        // k_sleep(DELAY_TIME);
        k_sleep(K_MSEC(500));
    }
}
