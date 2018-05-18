#!/usr/bin/env python3

import time
import statistics
import sqlite3
from datetime import datetime

from pyBusPirateLite import BitBang

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--steps', type=int, nargs='?', default=400, help='Steps to run through')
    parser.add_argument('-k', '--skip', type=int, nargs='?', default=2, help='Steps skip between measurements for shorter runtime')
    parser.add_argument('-d', '--database', default='spectra.sqlite3', help='sqlite3 database file to store results in')
    parser.add_argument('-w', '--wait', type=float, default=2.0, help='time to wait between samples in seconds')
    parser.add_argument('-o', '--oversample', type=int, default=32, help='oversampling ratio')
    parser.add_argument('-g', '--gain', type=float, default=None, help='Transimpedance gain of amplifier in MOhm')
    parser.add_argument('-c', '--comment', help='run comment')
    parser.add_argument('-p', '--port', default='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AD01W1RF-if00-port0', help='Serial port device of the control buspirate')
    parser.add_argument('run_name', nargs='?', default='auto')
    parser.add_argument('color', help='Captured color channel')
    args = parser.parse_args()

    db = sqlite3.connect(args.database)
    db.execute("""
        CREATE TABLE IF NOT EXISTS runs (
                capture_id INTEGER PRIMARY KEY,
                name TEXT,
                comment TEXT,
                color TEXT, -- Captured color channel
                gain REAL, -- Preamplifier transimpedance in Ohms
                timestamp REAL -- unix timestamp in fractional seconds
                )""")
    db.execute("""
        CREATE TABLE IF NOT EXISTS measurements (
                measurement_id INTEGER PRIMARY KEY,
                capture_id INTEGER,
                led_on INTEGER,
                step INTEGER,
                voltage REAL, -- volts
                voltage_stdev REAL, -- volts
                timestamp REAL, -- unix timestamp in fractional seconds
                FOREIGN KEY (capture_id) REFERENCES runs)""")

    class BPState:
        def __init__(self, port):
            self.bp = BitBang(port)
            self._led = 0
            self._stepper_dir = 'down'
            self.reinit()

        def reinit(self):
            self.bp.enter_bb()
            self.led(self._led)
            self.stepper_direction(self._stepper_dir)
            self.bp.cs = 0

        def led(self, st):
            self._led = st
            self.bp.mosi = st

        def stepper_direction(self, direction):
            self._stepper_dir = direction
            self.bp.aux = 0 if direction == 'down' else 1

        def step(self):
            self.bp.cs = 1
            time.sleep(0.005)
            self.bp.cs = 0
            time.sleep(0.005)

        def adc(self, oversampling):
            self.reinit()
            return [ self.bp.adc_value for _ in range(oversampling) ]

    bp = BPState(args.port)

    with db:
        cur = db.cursor()
        cur.execute('INSERT INTO runs(name, comment, color, gain, timestamp) VALUES (?, ?, ?, ?, ?)',
                (args.run_name, args.comment, args.color, args.gain*1e6, time.time()))
        capture_id = cur.lastrowid

    print('Starting capture {} "{}" at {:%y-%m-%d %H:%M:%S:%f}'.format(capture_id, args.run_name, datetime.now()))
    print('[measurement id] " " [step number] " " [reading (V)]')

    bp.stepper_direction('down')
    for _ in range(10):
        bp.step()

    bp.stepper_direction('up')
    for step in range(0, args.steps+args.skip, args.skip): # Run one skip past end to capture both interval boundaries
        for led_val in [1]: # This can be used for self-calibration.
            try:
                bp.led(led_val)
                time.sleep(args.wait)

                readings = bp.adc(args.oversample)
                mean, stdev = statistics.mean(readings), statistics.stdev(readings)

                with db:
                    cur = db.cursor()
                    cur.execute('''
                        INSERT INTO measurements (
                                capture_id, led_on, step, voltage, voltage_stdev, timestamp
                            ) VALUES (?, ?, ?, ?, ?, ?)''',
                            (capture_id, led_val, step, mean, stdev, time.time()))
                    print('{:08d} {:03} {}: {:5.4f} stdev {:5.4f}'.format(
                        cur.lastrowid, step, led_val, mean, stdev))
            except KeyboardInterrupt:
                raise
            except TypeError as e:
                print('Buspirate hiccup, ignoring:', e)
        for _ in range(args.skip):
            bp.step()

    bp.stepper_direction('down')
    for _ in range(args.steps+args.skip):
        bp.step()

