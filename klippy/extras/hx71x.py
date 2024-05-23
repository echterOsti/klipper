# HX711/HX717 Support
#
# Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import logging, struct
from . import bulk_sensor
from .bulk_adc_sensor import (TimestampHelper)

#
# Constants
#
BYTES_PER_SAMPLE = 4  # samples are 4 byte wide unsigned integers
MAX_SAMPLES_PER_MESSAGE = bulk_sensor.MAX_BULK_MSG_SIZE // BYTES_PER_SAMPLE
UPDATE_INTERVAL = 0.10


# Implementation of HX711 and HX717
class HX71xBase():
    def __init__(self, config,
                 sample_rate_options, default_sample_rate,
                 gain_options, default_gain):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        # Chip options
        dout_pin_name = config.get('dout_pin')
        sclk_pin_name = config.get('sclk_pin')
        ppins = printer.lookup_object('pins')
        dout_ppin = ppins.lookup_pin(dout_pin_name)
        sclk_ppin = ppins.lookup_pin(sclk_pin_name)
        self.mcu = mcu = dout_ppin['chip']
        self.oid = mcu.create_oid()
        if sclk_ppin['chip'] is not mcu:
            raise config.error("HX71x config error: All HX71x pins must be "
                               "connected to the same MCU")
        self.dout_pin = dout_ppin['pin']
        self.sclk_pin = sclk_ppin['pin']
        # Samples per second choices
        self.sps = config.getchoice('sample_rate', sample_rate_options,
                                    default=default_sample_rate)
        # set rest_ticks as a % of the sample_rate
        self.duty_cycle = config.getfloat('duty_cycle', minval=0.1,
                                          maxval=1.0, default=0.7)
        # gain/channel choices
        self.gain_channel = int(config.getchoice('gain', gain_options,
                                                 default=default_gain))
        ## Measurement conversion
        self._unpack_int32 = struct.Struct("<i").unpack_from
        ## Bulk Sensor Setup
        self.bulk_queue = bulk_sensor.BulkDataQueue(mcu, oid=self.oid)
        # Clock tracking
        chip_smooth = self.sps * UPDATE_INTERVAL * 2
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth, "<i")
        # Process messages in batches
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch, self._start_measurements,
            self._finish_measurements, UPDATE_INTERVAL)
        # publish raw samples to the socket
        self.batch_bulk.add_mux_endpoint("hx71x/dump_hx71x", "sensor",
                                         self.name,
                                         {'header': ('time', 'total_counts')})
        # Command Configuration
        mcu.add_config_cmd(
            "config_hx71x oid=%d gain_channel=%d dout_pin=%s sclk_pin=%s"
            % (self.oid, self.gain_channel, self.dout_pin, self.sclk_pin))
        mcu.add_config_cmd("query_hx71x oid=%d rest_ticks=0"
                                % (self.oid,), on_restart=True)

        mcu.register_config_callback(self._build_config)

    def _build_config(self):
        self.query_hx71x_cmd = self.mcu.lookup_command(
            "query_hx71x oid=%c rest_ticks=%u")
        self.ffreader.setup_query_command("query_hx71x_status oid=%c",
                                          oid=self.oid,
                                          cq=self.mcu.alloc_command_queue())

    def get_mcu(self):
        return self.mcu

    def get_samples_per_second(self):
        return self.sps

    # returns a tuple of the minimum and maximum value of the sensor, used to
    # detect if a data value is saturated
    def get_range(self):
        range_max = (2 ** 24)
        return -range_max, range_max

    # add_Client interface, direct pass through to bulk_sensor API
    def add_client(self, callback):
        self.batch_bulk.add_client(callback)

    # Start, stop, and process message batches
    def _start_measurements(self):
        # Start bulk reading
        rest_ticks = self.mcu.seconds_to_clock(self.duty_cycle / self.sps)
        self.query_hx71x_cmd.send([self.oid, rest_ticks])
        logging.info("HX71x starting '%s' measurements", self.name)
        # Initialize clock tracking
        self.ffreader.note_start()

    def _finish_measurements(self):
        # Halt bulk reading
        #TODO: this logs errors when klipper shuts down for other reasons
        self.query_hx71x_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info("HX71x finished '%s' measurements", self.name)

    def _process_batch(self, eventtime):
        samples = self.ffreader.pull_samples()
        if not samples:
            return {}
        return {'data': samples,
                'error_code': self.ffreader.get_last_overflows()}


class HX711(HX71xBase):
    def __init__(self, config):
        super(HX711, self).__init__(config,
                                    # HX711 sps options
                                    {80: 80, 10: 10}, 80,
                                    # HX711 gain/channel options
                                    {'A-128': 1, 'B-32': 2, 'A-64': 3}, 'A-128')


class HX717(HX71xBase):
    def __init__(self, config):
        super(HX717, self).__init__(config,
                                    # HX717 sps options
                                    {320: 320, 80: 80, 20: 20, 10: 10}, 320,
                                    # HX717 gain/channel options
                                    {'A-128': 1, 'B-64': 2, 'A-64': 3,
                                     'B-8': 4}, 'A-128')


HX71X_SENSOR_TYPES = {
    "hx711": HX711,
    "hx717": HX717
}
