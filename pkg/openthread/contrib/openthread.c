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
 * @brief       Implementation of OpenThread main functions
 *
 * @author      Jose Ignacio Alamos <jialamos@uc.cl>
 * @author      Hyung-Sin Kim <hs.kim@cs.berkeley.edu>
 * @}
 */

#include <assert.h>

#include "openthread/platform/uart.h"
#include "openthread/cli.h"
#include "openthread/ip6.h"
#include "openthread/instance.h"
#include "openthread/thread.h"
#include "openthread/platform/alarm-milli.h"
#include "openthread/tasklet.h"
#include "ot.h"
#include "random.h"
#include "thread.h"
#include "xtimer.h"

#ifdef MODULE_OPENTHREAD_NCP_FTD
#include "openthread/ncp.h"
#endif

#ifdef MODULE_AT86RF2XX
#include "at86rf2xx.h"
#include "at86rf2xx_params.h"
#endif

#define ENABLE_DEBUG (1)
#include "debug.h"

#ifdef MODULE_AT86RF2XX     /* is mutual exclusive with above ifdef */
#define OPENTHREAD_NETIF_NUMOF        (sizeof(at86rf2xx_params) / sizeof(at86rf2xx_params[0]))
#endif

static otInstance *sInstance;

#ifdef MODULE_AT86RF2XX
static at86rf2xx_t at86rf2xx_dev;
#endif

#define IEEE802154_ACK_LENGTH (5)
#define IEEE802154_DSN_OFFSET (2)

#define OPENTHREAD_QUEUE_LEN      (8)
static msg_t _queue[OPENTHREAD_QUEUE_LEN];
static char ot_main_thread_stack[THREAD_STACKSIZE_MAIN];
static kernel_pid_t main_pid;

static uint8_t rx_buf[OPENTHREAD_NETDEV_BUFLEN];
static uint8_t tx_buf[OPENTHREAD_NETDEV_BUFLEN];
static char ot_netdev_thread_stack[THREAD_STACKSIZE_MAIN];
static kernel_pid_t netdev_pid;

static char ot_timer_thread_stack[THREAD_STACKSIZE_MAIN];
static kernel_pid_t timer_pid;


/* get OpenThread instance */
otInstance* openthread_get_instance(void) {
    return sInstance;
}

/* get OpenThread thread main pid */
kernel_pid_t openthread_get_main_pid(void) {
    return main_pid;
}

/* get OpenThread thread netdev pid */
kernel_pid_t openthread_get_netdev_pid(void) {
    return netdev_pid;
}

/* get OpenThread thread timer pid */
kernel_pid_t openthread_get_timer_pid(void) {
    return timer_pid;
}

uint8_t ot_call_command(char* command, void *arg, void* answer) {
    ot_job_t job;

    job.command = command;
    job.arg = arg;
    job.answer = answer;

    msg_t msg, reply;
    msg.type = OPENTHREAD_JOB_MSG_TYPE_EVENT;
    msg.content.ptr = &job;
    msg_send_receive(&msg, &reply, openthread_get_netdev_pid());
    return (uint8_t)reply.content.value;
}

/* OpenThread will call this when switching state from empty tasklet to non-empty tasklet. */
void otTaskletsSignalPending(otInstance *aInstance) {
    (void) aInstance;
}

/* create a fake ACK frame */
// TODO: pass received ACK frame instead of generating one.
static inline otRadioFrame _create_fake_ack_frame(bool ackPending)
{
    otRadioFrame ackFrame;
    uint8_t psdu[IEEE802154_ACK_LENGTH];

    ackFrame.mPsdu = psdu;
    ackFrame.mLength = IEEE802154_ACK_LENGTH;
    ackFrame.mPsdu[0] = IEEE802154_FCF_TYPE_ACK;

    if (ackPending)
    {
        ackFrame.mPsdu[0] |= IEEE802154_FCF_FRAME_PEND;
    }

    ackFrame.mPsdu[1] = 0;
    ackFrame.mPsdu[2] = (openthread_get_txframe())->mPsdu[IEEE802154_DSN_OFFSET];

    ackFrame.mPower = OT_RADIO_RSSI_INVALID;

    return ackFrame;
}

/* Called upon TX event */
void sent_pkt(netdev_event_t event)
{
    otRadioFrame ackFrame;
    /* Tell OpenThread transmission is done depending on the NETDEV event */
    switch (event) {
        case NETDEV_EVENT_TX_COMPLETE:
            DEBUG("openthread: NETDEV_EVENT_TX_COMPLETE\n\n");
            ackFrame = _create_fake_ack_frame(false);
            otPlatRadioTxDone(sInstance, openthread_get_txframe(), &ackFrame, OT_ERROR_NONE);
            break;
        case NETDEV_EVENT_TX_COMPLETE_DATA_PENDING:
            DEBUG("openthread: NETDEV_EVENT_TX_COMPLETE_DATA_PENDING\n\n");
            ackFrame = _create_fake_ack_frame(true);
            otPlatRadioTxDone(sInstance, openthread_get_txframe(), &ackFrame, OT_ERROR_NONE);
            break;
        case NETDEV_EVENT_TX_NOACK:
            DEBUG("openthread: NETDEV_EVENT_TX_NOACK\n\n");
            otPlatRadioTxDone(sInstance, openthread_get_txframe(), NULL, OT_ERROR_NO_ACK);
            break;
        case NETDEV_EVENT_TX_MEDIUM_BUSY:
            DEBUG("openthread: NETDEV_EVENT_TX_MEDIUM_BUSY\n\n");
            otPlatRadioTxDone(sInstance, openthread_get_txframe(), NULL, OT_ERROR_CHANNEL_ACCESS_FAILURE);
            break;
        default:
            break;
    }
}


static void *_openthread_main_thread(void *arg) {
    main_pid = thread_getpid();

    /* enable OpenThread UART */
    otPlatUartEnable();

    /* init OpenThread */
    sInstance = otInstanceInitSingle();

    msg_init_queue(_queue, OPENTHREAD_QUEUE_LEN);
    msg_t msg, reply;
#if defined(MODULE_OPENTHREAD_CLI_FTD) || defined(MODULE_OPENTHREAD_CLI_MTD)
    otCliUartInit(sInstance);
    /* Init default parameters */
    otPanId panid = OPENTHREAD_PANID;
    uint8_t channel = OPENTHREAD_CHANNEL;
    otLinkSetPanId(sInstance, panid);
    otLinkSetChannel(sInstance, channel);
    /* Bring up the IPv6 interface  */
    otIp6SetEnabled(sInstance, true);
    /* Start Thread protocol operation */
    otThreadSetEnabled(sInstance, true);
#endif

#ifdef MODULE_OPENTHREAD_NCP_FTD
    otNcpInit(sInstance);
#endif

#if OPENTHREAD_ENABLE_DIAG
    diagInit(sInstance);
#endif

    ot_job_t *job;
    serial_msg_t* serialBuffer;
    while (1) {
        otTaskletsProcess(sInstance);
        if (otTaskletsArePending(sInstance) == false) {
            msg_receive(&msg);
            switch (msg.type) {
                case OPENTHREAD_NETDEV_MSG_TYPE_RECV:
                    /* Received an event from driver */
                    DEBUG("openthread_main: OPENTHREAD_NETDEV_MSG_TYPE_RECV received\n");
                    /* Tell OpenThread that receive has finished */
                    int res = *((int*)msg.content.ptr);
                    otPlatRadioReceiveDone(sInstance, res > 0 ? openthread_get_rxframe() : NULL, 
                                           res > 0 ? OT_ERROR_NONE : OT_ERROR_ABORT);
                    break;
                case OPENTHREAD_NETDEV_MSG_TYPE_SENT:
                    DEBUG("openthread_main: OPENTHREAD_NETDEV_MSG_TYPE_SENT received\n");
                    netdev_event_t event = *((netdev_event_t*)msg.content.ptr);
                    sent_pkt(event);
                    break;
                case OPENTHREAD_XTIMER_MSG_TYPE_EVENT:
                    /* Tell OpenThread a time event was received */
                    DEBUG("openthread_main: OPENTHREAD_XTIMER_MSG_TYPE_EVENT received\n");
                    otPlatAlarmMilliFired(sInstance);
                    break;
                case OPENTHREAD_SERIAL_MSG_TYPE_EVENT:
                    /* Tell OpenThread about the reception of a CLI command */
                    DEBUG("openthread_main: OPENTHREAD_SERIAL_MSG_TYPE_SEND received\n");
                    serialBuffer = (serial_msg_t*)msg.content.ptr;
                    otPlatUartReceived((uint8_t*) serialBuffer->buf,serialBuffer->length);
                    serialBuffer->serial_buffer_status = OPENTHREAD_SERIAL_BUFFER_STATUS_FREE;
                    break;
                case OPENTHREAD_JOB_MSG_TYPE_EVENT:
                    DEBUG("openthread_main: OPENTHREAD_JOB_MSG_TYPE_EVENT received\n");
                    job = msg.content.ptr;
                    reply.content.value = ot_exec_command(sInstance, job->command, job->arg, job->answer);
                    msg_reply(&msg, &reply);
                    break;
            }
        }
    }

    return NULL;
}

void openthread_bootstrap(void)
{
    /* init random */
    ot_random_init();

    /* setup netdev modules */
#ifdef MODULE_AT86RF2XX
    at86rf2xx_setup(&at86rf2xx_dev, &at86rf2xx_params[0]);
    netdev_t *netdev = (netdev_t *) &at86rf2xx_dev;
#endif
    openthread_radio_init(netdev, tx_buf, rx_buf);
    netdev_pid = openthread_netdev_init(ot_netdev_thread_stack, sizeof(ot_netdev_thread_stack), 
                           THREAD_PRIORITY_MAIN +3, "openthread_netdev", netdev);
    timer_pid = openthread_timer_init(ot_timer_thread_stack, sizeof(ot_timer_thread_stack),
                         THREAD_PRIORITY_MAIN +2, "openthread_timer"); 
    main_pid = thread_create(ot_main_thread_stack, sizeof(ot_main_thread_stack),
                         THREAD_PRIORITY_MAIN +1, THREAD_CREATE_STACKTEST,
                         _openthread_main_thread, NULL, "openthread_main");   
}
