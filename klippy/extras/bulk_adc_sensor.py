# Bulk ADC Sensor Support
#
# Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Helper for ClockSyncRegression that handles generating timestamps
# while processing a batch of samples
class TimestampHelper:
    def __init__(self, clock_sync, clock_updater, samples_per_msg):
        self.clock_sync = clock_sync
        self.last_sequence = clock_updater.get_last_sequence()
        self.time_base, self.chip_base, self.inv_freq = (
            clock_sync.get_time_translation())
        self.samples_per_msg = samples_per_msg
        self.msg_cdiff = None
        self.seq = 0
        self.last_msg_index = 0
    # updates the sequence when a new sequence number is reached
    # properly handles sequence number overflow
    def update_sequence(self, sequence):
        seq_diff = (sequence - self.last_sequence) & 0xffff
        seq_diff -= (seq_diff & 0x8000) << 1
        self.seq = self.last_sequence + seq_diff
        self.msg_cdiff = self.seq * self.samples_per_msg - self.chip_base
    # returns the time of a message by message index
    def time_of_msg(self, msg_index):
        self.last_msg_index = msg_index
        time = self.time_base + (self.msg_cdiff + msg_index) * self.inv_freq
        return round(time, 6)
    # update the chip clock when finished processing messages
    def set_last_chip_clock(self):
        chip_clock = self.seq * self.samples_per_msg + self.last_msg_index
        self.clock_sync.set_last_chip_clock(chip_clock)
