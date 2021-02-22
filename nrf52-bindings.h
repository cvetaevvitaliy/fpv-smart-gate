/*
 * Copyright (c) 2019, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SAMPLES_DRIVERS_LED_WS2812_H_
#define ZEPHYR_SAMPLES_DRIVERS_LED_WS2812_H_

/*
 * At 4 MHz, 1 bit is 250 ns, so 3 bits is 750 ns.
 *
 * That's cutting it a bit close to the edge of the timing parameters,
 * but it seems to work on AdaFruit LED rings.
 * 
 * //3*143 = 429ns   5*143 = 715ns  for WS281X
 * 
 * 		  _____
 * 		 |     |___|   01110000  high level
 * 		  ___
 * 		 |   |_____|   01000000  low level
 * 
 */

#define SPI_FREQ    4000000
#define ZERO_FRAME  0x40
#define ONE_FRAME   0x70

// #define ZERO_FRAME  0xC0
// #define ONE_FRAME   0xF0
// #define WS_HIGH 0XF8
// #define WS_LOW  0XE0

// #define ZERO_FRAME  0XE0
// #define ONE_FRAME   0XF8

#endif
