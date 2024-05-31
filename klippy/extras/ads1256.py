# ads1256 Support
#
# Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import logging, struct
from . import bulk_sensor, bus
from .bulk_sensor_adc import (BulkSensorAdc, LoadCellEndstopSensor,
                              TimestampHelper)

#
# Constants
#
BYTES_PER_SAMPLE = 4  # samples are 4 byte wide unsigned integers
MAX_SAMPLES_PER_MESSAGE = bulk_sensor.MAX_BULK_MSG_SIZE // BYTES_PER_SAMPLE
UPDATE_INTERVAL = 0.10
RESET_CMD = 0xFE
START_SYNC_CMD = 0xFC
RREG_CMD_FIRSTBYTE = 0x10 # The last 4 bits are the register address
RREG_CMD_SECONDBYTE = 0x00 # This byte is the number of bytes to read - 1
WREG_CMD_FIRSTBYTE = 0x50 # The last 4 bits are the register address
WREG_SECONDBYTE = 0x00 # This byte is the number of bytes to write - 1
RDATAC_CMD = 0x03
SYNC_CMD = 0xFC
WAKEUP_CMD = 0xFF
NOOP_CMD = 0x0

# turn bytearrays into pretty hex strings: [0xff, 0x1]
def hexify(byte_array):
    return "[%s]" % (", ".join([hex(b) for b in byte_array]))

class ads1256(BulkSensorAdc, LoadCellEndstopSensor):
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.unpack_int = struct.Struct('<i').unpack_from
        ## Process config options
        # Gain
        self.gain_options = {'1': 0x0, '2': 0x1, '4': 0x2, '8': 0x3, '16': 0x4,
                             '32': 0x5, '64': 0x6}
        self.gain = config.getchoice('gain', self.gain_options, default='64')
        # Sample rate
        self.sps_normal = {'2.5': 0x3, '5': 0x13, '10': 0x23, '15': 0x33, '25': 0x43,
                            '30': 0x53, '50': 0x63, '60': 0x72, '100': 0x82, '500': 0x92, 
                            '1000': 0xA1, '2000': 0xB0, '3750': 0xC0, '7500': 0xD0,
                            '15000': 0xE0, '30000': 0xF0}
        self.sps_turbo = {'0':0x0}
        self.sps_options = self.sps_normal.copy()
        self.sps_options.update(self.sps_turbo)
        self.sps = config.getchoice('sps', self.sps_options, default='2000')
        self.is_turbo = str(self.sps) in self.sps_turbo
        ## SPI Setup (needs to know turbo mode to set this up)
        spi_speed = (512 * 1000)
        if self.is_turbo:
            spi_speed *= 2
        self.spi = bus.MCU_SPI_from_config(config, 1, default_speed=spi_speed)
        self.mcu = mcu = self.spi.get_mcu()
        self.oid = mcu.create_oid()
        # DRDY Pin (needs SPI MCU to set this up)
        drdy_pin = config.get('data_ready_pin')
        ppins = printer.lookup_object('pins')
        drdy_ppin = ppins.lookup_pin(drdy_pin)
        self.data_ready_pin = drdy_ppin['pin']
        drdy_pin_mcu = drdy_ppin['chip']
        if drdy_pin_mcu != self.mcu:
            raise config.error("ads1256 config error: SPI communication and"
                               " data_ready_pin must be on the same MCU")
        ## Bulk Sensor Setup
        self.bulk_queue = bulk_sensor.BulkDataQueue(self.mcu, oid=self.oid)
        # Clock tracking
        chip_smooth = self.sps * UPDATE_INTERVAL * 2
        self.clock_sync = bulk_sensor.ClockSyncRegression(self.mcu, chip_smooth)
        self.clock_updater = bulk_sensor.ChipClockUpdater(self.clock_sync,
                                                          BYTES_PER_SAMPLE)
        # Process messages in batches
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch, self._start_measurements,
            self._finish_measurements, UPDATE_INTERVAL)
        # publish raw samples to the socket
        self.batch_bulk.add_mux_endpoint("ads1256/dump_ads1256", "sensor",
                                         self.name,
                                         {'header': ('time', 'counts')})
        self.query_ads1256_cmd = None
        self.mcu.register_config_callback(self._build_config)
        # startup, when klippy is ready, start capturing data
        printer.register_event_handler("klippy:ready", self._handle_ready)

    def _build_config(self):
        cmdqueue = self.spi.get_command_queue()
        self.mcu.add_config_cmd("config_ads1256 oid=%d spi_oid=%d "
            "data_ready_pin=%s"
            % (self.oid, self.spi.get_oid(), self.data_ready_pin))
        self.config_endstop_cmd = self.mcu.lookup_command(
            "attach_endstop_ads1256 oid=%c load_cell_endstop_oid=%c")
        self.mcu.add_config_cmd("query_ads1256 oid=%d rest_ticks=0"
                                % (self.oid,), on_restart=True)
        self.query_ads1256_cmd = self.mcu.lookup_command(
            "query_ads1256 oid=%c rest_ticks=%u", cq=cmdqueue)
        self.clock_updater.setup_query_command(self.mcu,
                                               "query_ads1256_status oid=%c",
                                               self.oid, cq=cmdqueue)

    def _handle_ready(self):
        # reset chip on startup
        self.reset_chip()
        reactor = self.printer.get_reactor()
        reactor.pause(reactor.monotonic() + 0.1)
        self.setup_chip()

    def get_mcu(self):
        return self.mcu

    def get_samples_per_second(self):
        return self.sps

    # returns a tuple of the minimum and maximum value of the sensor, used to
    # detect if a data value is saturated
    def get_range(self):
        range_max = (2 ** 24)
        return -range_max, range_max

    # add_client interface, direct pass through to bulk_sensor API
    def add_client(self, callback):
        self.batch_bulk.add_client(callback)

    def attach_endstop(self, endstop_oid):
        self.config_endstop_cmd.send_wait_ack([self.oid, endstop_oid])

    # Measurement decoding
    def _extract_samples(self, raw_blocks):
        # Process every message in capture_buffer
        unpack_int = self.unpack_int
        max_samples = (len(raw_blocks) * MAX_SAMPLES_PER_MESSAGE)
        samples = collections.deque(maxlen=max_samples)
        timestamps = TimestampHelper(self.clock_sync, self.clock_updater,
                                     MAX_SAMPLES_PER_MESSAGE)
        for block in raw_blocks:
            timestamps.update_sequence(block['sequence'])
            data = bytearray(block['data'])
            for i in range(len(data) // BYTES_PER_SAMPLE):
                counts = unpack_int(data, BYTES_PER_SAMPLE * i)[0]
                samples.append((timestamps.time_of_msg(i), counts))
        timestamps.set_last_chip_clock()
        return list(samples)

    # Start, stop, and process message batches
    def _start_measurements(self):
        # Start bulk reading
        self.bulk_queue.clear_samples()
        rest_ticks = self.mcu.seconds_to_clock(0.7 / float(self.sps))
        self.query_ads1256_cmd.send([self.oid, rest_ticks])
        logging.info("ads1256 starting '%s' measurements", self.name)
        # Initialize clock tracking
        self.clock_updater.note_start()

    def _finish_measurements(self):
        # Halt bulk reading
        self.query_ads1256_cmd.send_wait_ack([self.oid, 0])
        self.bulk_queue.clear_samples()
        logging.info("ads1256 finished '%s' measurements", self.name)

    def _process_batch(self, eventtime):
        self.clock_updater.update_clock()
        raw_samples = self.bulk_queue.pull_samples()
        if not raw_samples:
            return {}
        samples = self._extract_samples(raw_samples)
        if not samples:
            return {}
        return {'data': samples}

    def reset_chip(self):
        self.send_command(RESET_CMD)

    def setup_chip(self):
        # This function will set up the chip with gain, sample rate.
        
        sps_list = self.sps_normal
        data_rate = list(sps_list.keys()).index(str(self.sps))
        PGA_byte = (0x20 | self.gain)
        reg_values = [PGA_byte, data_rate]
        self.write_reg(0x02, reg_values)
        # Syncronize ADC conversion
        self.send_command(SYNC_CMD)
        self.send_command(WAKEUP_CMD)
        # Start continuous conversion
        self.send_command(RDATAC_CMD)

        # ADS1220 OLD CODE
        """
        continuous = 0x1  # enable continuous conversions
        mode = 0x2 if self.is_turbo else 0x0  # turbo mode
        sps_list = self.sps_normal
        data_rate = list(sps_list.keys()).index(str(self.sps))
        reg_values = [(self.gain << 1),
                      (data_rate << 5) | (mode << 3) | (continuous << 2)]
        self.write_reg(0x0, reg_values)
        # start measurements immediately
        self.send_command(START_SYNC_CMD)
        """
        
    def read_reg(self, reg, byte_count):
        read_command = [RREG_CMD_FIRSTBYTE | reg, RREG_CMD_SECONDBYTE | byte_count-1]
        read_command += [NOOP_CMD] * byte_count
        params = self.spi.spi_transfer(read_command)
        return bytearray(params['response'][1:])

    def send_command(self, cmd):
        self.spi.spi_send([cmd])

    def write_reg(self, reg, register_bytes):
        #write_command = [WREG_CMD | (reg << 2) | (len(register_bytes) - 1)]
        
        # This will have to be adapted for the ADS1256, as it uses 2 Bytes for the WREG command.
        # Therefore we will make sure to have write_command be in this format 0101 rrrr 0000 nnnn where rrrr is the register address and nnnn is the number of bytes to write - 1
        # This will be done in two lines: The first line will put the value of the register address in the smaller 4 bits of the first byte, and the second line will put the number of bytes to write - 1 in the smaller 4 bits of the second byte.
        write_command = [WREG_CMD_FIRSTBYTE | reg, WREG_SECONDBYTE | len(register_bytes) - 1]
        write_command.extend(register_bytes)
        self.spi.spi_send(write_command)
        stored_val = self.read_reg(reg, len(register_bytes))
        stored_hex = hexify(stored_val)
        val_hex = hexify(register_bytes)
        if val_hex != stored_hex:
            raise self.printer.command_error(
                "Failed to set ads1256 register [0x%x] to %s: got %s. "
                "This may be a connection problem (e.g. faulty wiring)" % (
                    reg, val_hex, stored_hex))

ads1256_SENSOR_TYPE = {"ads1256": ads1256}
