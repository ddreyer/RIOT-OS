/*
 * Copyright (C) 2018 UC Berkeley
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_hamilton
 * @ingroup     boards
 * @brief       Support for the Hamilton board.
 * @{
 *
 * @file
 * @brief       Board specific definitions for the Hamilton
 *              board
 *
 * @author      Michael Andersen <m.andersen@cs.berkeley.edu>
 * @author      Hyung-Sin Kim <hs.kim@cs.berkeley.edu>
 */

#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"
#include "periph_conf.h"
#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    xtimer configuration
 * @{
 */
#define XTIMER_DEV          TIMER_DEV(1)
#define XTIMER_CHAN         (0)
/** @} */

/**
 * @name AT86RF233 configuration
 * @{
 */
#define AT86RF2XX_PARAM_SPI      SPI_DEV(0)
#define AT86RF2XX_PARAM_SPI_CLK  SPI_CLK_5MHZ
#define AT86RF2XX_PARAM_CS       GPIO_PIN(PB, 31)
#define AT86RF2XX_PARAM_INT      GPIO_PIN(PB, 0)
#define AT86RF2XX_PARAM_SLEEP    GPIO_PIN(PA, 20)
#define AT86RF2XX_PARAM_RESET    GPIO_PIN(PB, 15)
/** @} */

/**
 * @name    LED pin definitions and handlers
 * @{
 */
#define LED_PIN                  GPIO_PIN(0, 19)

#define LED_PORT                 PORT->Group[0]
#define LED_MASK                 (1 << 19)

#define LED_ON                   (LED_PORT.OUTCLR.reg = LED_MASK)
#define LED_OFF                  (LED_PORT.OUTSET.reg = LED_MASK)
#define LED_TOGGLE               (LED_PORT.OUTTGL.reg = LED_MASK)
/** @} */

/**
 * @name    Button pin definitions
 * @{
 */
#define BTN_PORT                 PORT->Group[0]
#define BTN_PIN                  GPIO_PIN(0, 18)
#define BTN_MODE                 GPIO_IN_PU
/** @} */

/**
 * @name HDC1080 configuration
 * @{
 */
#define HDC1000_PARAM_I2C        I2C_DEV(0)
#define HDC1000_PARAM_ADDR       (0x40)
#ifndef HDC1000_PARAM_RES
#define HDC1000_PARAM_RES        HDC1000_14BIT
#endif
/** @} */

/**
 * @name TMP006 configuration
 * @{
 */
#define TMP006_PARAM_I2C         I2C_DEV(0)
#define TMP006_PARAM_ADDR        (0x44)
#define TMP006_PARAM_RATE        TMP006_CONFIG_CR_AS2
#define TMP006_USE_LOW_POWER     (1)
/** @} */

/**
 * @name PULSE_COUNTER configuration
 * @{
 */
#define PULSE_COUNTER_GPIO       BTN_PIN
#define PULSE_COUNTER_GPIO_FLANK GPIO_FALLING
/** @} */

/**
 * @name ToDo: APDS9007, FXOS8700, PIR configuration
 * @{
 */
/** @} */

/**
 * @name Factory-generated parameters
 * @{
 */
extern const uint64_t* const fb_sentinel;
extern const uint64_t* const fb_flashed_time;
extern const uint8_t*  const fb_eui64;
extern const uint16_t* const fb_device_id;
extern const uint64_t* const fb_designator;
extern const uint8_t*  const fb_aes128_key;
extern const uint8_t*  const fb_25519_pub;
extern const uint8_t*  const fb_25519_priv;
#define FB_SENTINEL_VALUE 0x27c83f60f6b6e7c8
#define HAS_FACTORY_BLOCK (*fb_sentinel == FB_SENTINEL_VALUE)
/** @} */

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
