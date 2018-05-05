#!/usr/bin/env python3

import time
import statistics
import sqlite3
from olsndot import Olsndot, Driver
from datetime import datetime

from pyBusPirateLite import BitBang

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('run_name', nargs='?', default='auto')
    parser.add_argument('buspirate_port', nargs='?', default='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AD01W1RF-if00-port0')
    parser.add_argument('-s', '--steps', type=int, nargs='?', default=100, help='Steps to run through')
    parser.add_argument('-d', '--database', default='spectra.sqlite3', help='sqlite3 database file to store results in')
    parser.add_argument('-w', '--wait', type=float, default=0.1, help='time to wait between samples in seconds')
    parser.add_argument('-o', '--oversample', type=int, default=16, help='oversampling ratio')
    args = parser.parse_args()

    db = sqlite3.connect(args.database)
    db.execute("""
        CREATE TABLE IF NOT EXISTS runs (
                run_id INTEGER PRIMARY KEY,
                name TEXT,
                comment TEXT,
                timestamp REAL -- unix timestamp in fractional seconds
                )""")
    db.execute("""
        CREATE TABLE IF NOT EXISTS measurements (
                measurement_id INTEGER PRIMARY KEY,
                run_id INTEGER,
                led_on INTEGER,
                step INTEGER,
                voltage REAL, -- volts
                voltage_stdev REAL, -- volts
                timestamp REAL, -- unix timestamp in fractional seconds
                FOREIGN KEY (run_id) REFERENCES runs)""")

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

    bp = BPState(args.buspirate_port)

    run_name = args.run_name
    if not str.isnumeric(args.run_name[-1]):
        names = [ n[len(run_name):] for n, in db.execute(
            'SELECT name FROM runs WHERE name LIKE ?||"%"', (run_name,)).fetchall() ]
        names.append('0') # in case we get no results
        run_name += str(1+max(int(n) if str.isnumeric(n) else 0 for n in names))
    with db:
        cur = db.cursor()
        cur.execute('INSERT INTO runs(name, timestamp) VALUES (?, ?)',
                (run_name, time.time()))
        run_id = cur.lastrowid

    print('Starting run {} "{}" at {:%y-%m-%d %H:%M:%S:%f}'.format(run_id, run_name, datetime.now()))
    print('[measurement id] " " [step number] " " [reading (V)]')

    bp.stepper_direction('down')
    for _ in range(10):
        bp.step()

    bp.stepper_direction('up')
    for step in range(args.steps):
        bp.step()
        for led_val in [0, 1]:
            try:
                bp.led(led_val)
                time.sleep(args.wait)

                readings = bp.adc(args.oversample)
                mean, stdev = statistics.mean(readings), statistics.stdev(readings)

                with db:
                    cur = db.cursor()
                    cur.execute('''
                        INSERT INTO measurements (
                                run_id, led_on, step, voltage, voltage_stdev, timestamp
                            ) VALUES (?, ?, ?, ?, ?, ?)''',
                            (run_id, led_val, step, mean, stdev, time.time()))
                    print('{:08d} {:03} {}: {:5.4f} stdev {:5.4f}'.format(
                        cur.lastrowid, step, led_val, mean, stdev))
            except KeyboardInterrupt:
                raise
            except TypeError as e:
                print('Buspirate hiccup, ignoring:', e)

    bp.stepper_direction('down')
    for _ in range(args.steps):
        bp.step()

