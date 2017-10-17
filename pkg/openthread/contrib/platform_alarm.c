/*
 * Copyright (C) 2017 Fundacion Inria Chile
 * Copyright (C) 2017 UC Berkeley
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net
 * @file
 * @brief       Implementation of OpenThread alarm platform abstraction
 *
 * @author      Jose Ignacio Alamos <jialamos@uc.cl>
 * @author      Hyung-Sin Kim <hs.kim@cs.berkeley.edu>
 * @}
 */

#include <stdint.h>

#include "msg.h"
#include "openthread/platform/alarm-milli.h"
#include "openthread/tasklet.h"
#include "ot.h"
#include "thread.h"
#include "xtimer.h"
#include "timex.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define OPENTHREAD_TIMER_QUEUE_LEN      (2)
static msg_t _queue[OPENTHREAD_TIMER_QUEUE_LEN];
static kernel_pid_t timer_pid;

static xtimer_t ot_timer;
static msg_t ot_alarm_msg;

/**
 * Set the alarm to fire at @p aDt milliseconds after @p aT0.
 *
 * @param[in] aInstance  The OpenThread instance structure.
 * @param[in] aT0        The reference time.
 * @param[in] aDt        The time delay in milliseconds from @p aT0.
 */
void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    DEBUG("openthread_main: otPlatAlarmMilliStartAt: aT0: %" PRIu32 ", aDT: %" PRIu32 "pid %u\n", aT0, aDt, event_pid);
    printf("[timer set] %lu ms\n", aDt);
    ot_alarm_msg.type = OPENTHREAD_XTIMER_MSG_TYPE_EVENT;
    if (aDt == 0) {
        msg_send(&ot_alarm_msg, timer_pid);
    }
    else {
        int dt = aDt * US_PER_MS;
        xtimer_set_msg(&ot_timer, dt, &ot_alarm_msg, timer_pid);
    }
}

/* OpenThread will call this to stop alarms */
void otPlatAlarmMilliStop(otInstance *aInstance)
{
    DEBUG("openthread_main: otPlatAlarmMilliStop\n");
    xtimer_remove(&ot_timer);
}

/* OpenThread will call this for getting running time in millisecs */
uint32_t otPlatAlarmMilliGetNow(void)
{
    uint32_t now = xtimer_now_usec() / US_PER_MS;
    DEBUG("openthread_main: otPlatAlarmMilliGetNow: %" PRIu32 "\n", now);
    return now;
}

/* OpenThread Timer Thread */
static void *_openthread_timer_thread(void *arg) {
    timer_pid = thread_getpid();

    msg_init_queue(_queue, OPENTHREAD_TIMER_QUEUE_LEN);
    msg_t msg;

    while (1) {
        msg_receive(&msg);            
        switch (msg.type) {
            case OPENTHREAD_XTIMER_MSG_TYPE_EVENT:
                /* Tell OpenThread a time event was received */
                DEBUG("openthread_timer: OPENTHREAD_XTIMER_MSG_TYPE_EVENT received\n");
                msg.type = OPENTHREAD_XTIMER_MSG_TYPE_EVENT;
                msg_send(&msg, openthread_get_main_pid());
                break;
        }
    }

    return NULL;
}

/* starts OpenThread timer thread */
int openthread_timer_init(char *stack, int stacksize, char priority, const char *name) {

    timer_pid = thread_create(stack, stacksize, priority, THREAD_CREATE_STACKTEST,
                         _openthread_timer_thread, NULL, name);

    if (timer_pid <= 0) {
        return -EINVAL;
    }

    return timer_pid;
}
