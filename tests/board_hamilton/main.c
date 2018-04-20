/*
 * Copyright (C) 2018 UC Berkeley
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Test the Hamilton board
 *
 * @author      Hyung-Sin Kim <hs.kim@cs.berkeley.edus>
 * @author      Michael Andersen <m.andersen@cs.berkeley.edus>
 *
 * @}
 */

#include <stdio.h>
#include <rtt_stdio.h>
#include "saul_reg.h"
#include "phydat.h"
#include "xtimer.h"

#define SAMPLE_INTERVAL  (2*US_PER_SEC)

saul_reg_t *sensor_pulsecnt_t  = NULL;
saul_reg_t *sensor_temp_t    = NULL;
saul_reg_t *sensor_hum_t     = NULL;
saul_reg_t *sensor_objtemp_t = NULL;

int sensor_config(void) {
    sensor_pulsecnt_t  = saul_reg_find_type(SAUL_SENSE_COUNT);
    if (sensor_pulsecnt_t == NULL) {
        puts("[ERROR] Failed to init BUTTON sensor\n");
        return 1;
    } else {
        puts("BUTTON sensor OK\n");
    }

    sensor_hum_t     = saul_reg_find_type(SAUL_SENSE_HUM);
    if (sensor_hum_t == NULL) {
        puts("[ERROR] Failed to init HUM sensor\n");
        return 1;
    } else {
        puts("HUM sensor OK\n");
    }

    sensor_temp_t    = saul_reg_find_type(SAUL_SENSE_TEMP);
    if (sensor_temp_t == NULL) {
        puts("[ERROR] Failed to init TEMP sensor\n");
        return 1;
    } else {
        puts("TEMP sensor OK\n");
    }

    sensor_objtemp_t = saul_reg_find_type(SAUL_SENSE_OBJTEMP);
    if (sensor_objtemp_t == NULL) {
        puts("[ERROR] Failed to init RADTEMP sensor\n");
        return 1;
    } else {
        puts("TEMP sensor OK\n");
    }

    return 0;
}

void sensor_sample(void) {
    phydat_t output; /* Sensor output data (maximum 3-dimension)*/
    int dim;         /* Demension of sensor output */

    /* Push button events 1-dim */
    dim = saul_reg_read(sensor_pulsecnt_t, &output);
    if (dim > 0) {
        printf("\nDev: %s\tType: %s\n", sensor_pulsecnt_t->name,
                 saul_class_to_str(sensor_pulsecnt_t->driver->type));
        phydat_dump(&output, dim);
    } else {
        puts("[ERROR] Failed to read button events\n");
    }

    /* Temperature 1-dim */
    dim = saul_reg_read(sensor_temp_t, &output);
    if (dim > 0) {
        printf("\nDev: %s\tType: %s\n", sensor_temp_t->name,
                 saul_class_to_str(sensor_temp_t->driver->type));
        phydat_dump(&output, dim);
    } else {
        puts("[ERROR] Failed to read Temperature\n");
    }

    /* Humidity 1-dim */
    dim = saul_reg_read(sensor_hum_t, &output); /* 15ms */
    if (dim > 0) {
        printf("\nDev: %s\tType: %s\n", sensor_hum_t->name,
                 saul_class_to_str(sensor_hum_t->driver->type));
        phydat_dump(&output, dim);
    } else {
        puts("[ERROR] Failed to read Humidity\n");
    }

    /* Radient temperature 1-dim */
    dim = saul_reg_read(sensor_objtemp_t, &output);
    if (dim > 0) {
        printf("\nDev: %s\tType: %s\n", sensor_objtemp_t->name,
                 saul_class_to_str(sensor_objtemp_t->driver->type));
        phydat_dump(&output, dim);
    } else {
        puts("[ERROR] Failed to read Radient Temperature\n");
    }
}

int main(void)
{
    puts("Welcome to RIOT!\n");
    puts("Please refer to the README.md for more information about this app\n");

    if (sensor_config()) {
        puts("[ERROR] Failed to initialize sensors\n");
        return 1;
    }

    uint16_t count = 0;

    while (1) {
        xtimer_usleep(SAMPLE_INTERVAL);
        printf("\n############# HAMILTON SENSING RESULTS [%u] #############\n", ++count);
        sensor_sample();
        LED_TOGGLE;
    }

    return 0;
}
