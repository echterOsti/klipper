// Support for bit-banging commands to HX711 and HX717 ADC chips
//
// Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_AVR
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_add_timer
#include "sensor_bulk.h" // sensor_bulk_report
#include <stdint.h>

struct hx71x_adc {
    uint8_t gain_channel;   // the gain+channel selection (1-4)
    uint8_t pending_flag;
    uint32_t rest_ticks;
    struct timer timer;
    struct gpio_in dout; // pin used to receive data from the hx71x
    struct gpio_out sclk; // pin used to generate clock for the hx71x
    struct sensor_bulk sb;
};

// Flag types
enum {FLAG_PENDING = 1 << 0};

#define BYTES_PER_SAMPLE 4
#define SAMPLE_ERROR 0x80000000
#define ERROR_READY_AFTER_READ 1
#define ERROR_READ_TOO_LONG 2
#define ERROR_OUT_OF_RANGE 3

static struct task_wake wake_hx71x;

/****************************************************************
 * Timing
 ****************************************************************/

typedef unsigned int hx71x_time_t;

static hx71x_time_t
nsecs_to_ticks(uint32_t ns)
{
    return timer_from_us(ns * 1000) / 1000000;
}

static inline int
hx71x_check_elapsed(hx71x_time_t t1, hx71x_time_t t2
                       , hx71x_time_t ticks)
{
    return t2 - t1 >= ticks;
}

// The AVR micro-controllers require specialized timing
#if CONFIG_MACH_AVR

#include <avr/interrupt.h> // TCNT1

static hx71x_time_t
hx71x_get_time(void)
{
    return TCNT1;
}

#define hx71x_delay_no_irq(start, ticks) (void)(ticks)
#define hx71x_delay(start, ticks) (void)(ticks)

#else

static hx71x_time_t
hx71x_get_time(void)
{
    return timer_read_time();
}

static inline void
hx71x_delay_no_irq(hx71x_time_t start, hx71x_time_t ticks)
{
    while (!hx71x_check_elapsed(start, hx71x_get_time(), ticks))
        ;
}

static inline void
hx71x_delay(hx71x_time_t start, hx71x_time_t ticks)
{
    while (!hx71x_check_elapsed(start, hx71x_get_time(), ticks))
        irq_poll();
}

#endif

/****************************************************************
 * HX711 and HX717 Sensor Support
 ****************************************************************/
// both HX717 and HX711 have 200ns min pulse time for clock pin on/off
#define MIN_PULSE_TIME nsecs_to_ticks(200)
#define MAX_READ_TIME timer_from_us(50)

// Event handler that wakes wake_hx71x() periodically
static uint_fast8_t
hx71x_event(struct timer *timer)
{
    struct hx71x_adc *hx71x = container_of(timer, struct hx71x_adc, timer);
    hx71x->pending_flag = 1;
    sched_wake_task(&wake_hx71x);
    return SF_DONE;
}

// Helper code to reschedule the hx71x_event() timer
static void
hx71x_reschedule_timer(struct hx71x_adc *hx71x)
{
    irq_disable();
    hx71x->timer.waketime = timer_read_time() + hx71x->rest_ticks;
    sched_add_timer(&hx71x->timer);
    irq_enable();
}

int8_t
hx71x_is_data_ready(struct hx71x_adc *hx71x) {
    // if the dout pin is high the sample is not ready
    return !gpio_in_read(hx71x->dout);
}

// Add a measurement to the buffer
static void
add_sample(struct hx71x_adc *hx71x, uint_fast32_t counts)
{
    hx71x->sb.data[hx71x->sb.data_count] = counts;
    hx71x->sb.data[hx71x->sb.data_count + 1] = counts >> 8;
    hx71x->sb.data[hx71x->sb.data_count + 2] = counts >> 16;
    hx71x->sb.data[hx71x->sb.data_count + 3] = counts >> 24;
    hx71x->sb.data_count += BYTES_PER_SAMPLE;
}

static void
flush_samples(struct hx71x_adc *hx71x, uint8_t oid)
{
    if (hx71x->sb.data_count + BYTES_PER_SAMPLE > ARRAY_SIZE(hx71x->sb.data))
        sensor_bulk_report(&hx71x->sb, oid);
}

// log the error to the data stream and flush
static void
send_error(struct hx71x_adc *hx71x, uint8_t oid, uint8_t error_code) {
    hx71x->pending_flag = 0;
    // write error codes and send to host
    add_sample(hx71x, SAMPLE_ERROR);
    hx71x->sb.possible_overflows = error_code;
    sensor_bulk_report(&hx71x->sb, oid);
}

// Pulse all clock pins to move to the next bit
inline static void
hx71x_pulse_clocks(struct hx71x_adc *hx71x) {
    irq_disable();
    gpio_out_write(hx71x->sclk, 1);
    hx71x_delay_no_irq(hx71x_get_time(), MIN_PULSE_TIME);
    gpio_out_write(hx71x->sclk, 0);
    hx71x_delay_no_irq(hx71x_get_time(), MIN_PULSE_TIME);
    irq_enable();
}

// hx71x ADC query
void
hx71x_read_adc(struct hx71x_adc *hx71x, uint8_t oid)
{
    if (!hx71x_is_data_ready(hx71x)) {
        hx71x_reschedule_timer(hx71x);
        return;
    }

    // data is ready
    int32_t counts = 0;
    hx71x_time_t start_time = timer_read_time();
    for (uint_fast8_t sample_idx = 0; sample_idx < 24; sample_idx++) {
        hx71x_pulse_clocks(hx71x);
        // read 2's compliment int bits
        counts = (counts << 1) | gpio_in_read(hx71x->dout);
    }

    // bit bang 1 to 4 more bits to configure gain & channel for the next sample
    for (uint8_t gain_idx = 0; gain_idx < hx71x->gain_channel; gain_idx++) {
        hx71x_pulse_clocks(hx71x);
    }

    if (hx71x_is_data_ready(hx71x)) {
        send_error(hx71x, oid, ERROR_READY_AFTER_READ);
        return;
    }

    hx71x_time_t time_diff = timer_read_time() - start_time;
    if (time_diff >= MAX_READ_TIME) {
        send_error(hx71x, oid, ERROR_READ_TOO_LONG);
        return;
    }

    // handle -0 value
    if (counts == 0x800000) {
        counts = 0;
    }
    else if (counts > 0x800000) {
        // extend 2's complement 24 bits to 32bits
        counts |= 0xFF000000;
    }

    if (counts < -0x7FFFFF || counts > 0x7FFFFF) {
        send_error(hx71x, oid, ERROR_OUT_OF_RANGE);
        return;
    }

    add_sample(hx71x, counts);
    flush_samples(hx71x, oid);
    hx71x->pending_flag = 0;
    hx71x_reschedule_timer(hx71x);
}

// Create a hx71x sensor
void
command_config_hx71x(uint32_t *args)
{
    struct hx71x_adc *hx71x = oid_alloc(args[0]
                , command_config_hx71x, sizeof(*hx71x));
    hx71x->timer.func = hx71x_event;
    hx71x->pending_flag = 0;
    uint8_t gain_channel = args[1];
    if (gain_channel < 1 || gain_channel > 4) {
        shutdown("HX71x gain/channel out of range 1-4");
    }
    hx71x->gain_channel = gain_channel;
    hx71x->dout = gpio_in_setup(args[2], 1);
    hx71x->sclk = gpio_out_setup(args[3], 0);
    gpio_out_write(hx71x->sclk, 1); // put chip in power down state
}
DECL_COMMAND(command_config_hx71x, "config_hx71x oid=%c gain_channel=%c"
    " dout_pin=%u sclk_pin=%u");

// start/stop capturing ADC data
void
command_query_hx71x(uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx71x_adc *hx71x = oid_lookup(oid, command_config_hx71x);
    sched_del_timer(&hx71x->timer);
    hx71x->pending_flag = 0;
    hx71x->rest_ticks = args[1];
    if (!hx71x->rest_ticks) {
        // End measurements
        gpio_out_write(hx71x->sclk, 1); // put chip in power down state
        return;
    }
    // Start new measurements
    gpio_out_write(hx71x->sclk, 0); // wake chip from power down
    sensor_bulk_reset(&hx71x->sb);
    hx71x_reschedule_timer(hx71x);
}
DECL_COMMAND(command_query_hx71x,
             "query_hx71x oid=%c rest_ticks=%u");

void
command_query_hx71x_status(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx71x_adc *hx71x = oid_lookup(oid, command_config_hx71x);
    irq_disable();
    const uint32_t start_t = timer_read_time();
    uint8_t is_data_ready = hx71x_is_data_ready(hx71x);
    irq_enable();
    uint8_t pending_bytes = is_data_ready ? BYTES_PER_SAMPLE : 0;
    sensor_bulk_status(&hx71x->sb, oid, start_t, 0, pending_bytes);
}
DECL_COMMAND(command_query_hx71x_status, "query_hx71x_status oid=%c");

// Background task that performs measurements
void
hx71x_capture_task(void)
{
    if (!sched_check_wake(&wake_hx71x))
        return;
    uint8_t oid;
    struct hx71x_adc *hx71x;
    foreach_oid(oid, hx71x, command_config_hx71x) {
        if (hx71x->pending_flag) {
            hx71x_read_adc(hx71x, oid);
        }
    }
}
DECL_TASK(hx71x_capture_task);
