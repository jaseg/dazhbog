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
    parser.add_argument('buspirate_port', nargs='?', default='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AD01W1RF-if00-port0')
    args = parser.parse_args()

    bp = BitBang(args.buspirate_port)
    bp.enter_bb()
    bp.mosi = 1

    def stepper_direction_down():
        bp.aux = 0

    def stepper_direction_up():
        bp.aux = 1

    def stepper_step():
        bp.cs = 1
        #time.sleep(0.005)
        bp.cs = 0
        #time.sleep(0.005)

    import curses
    screen = curses.initscr()
    curses.noecho()
    curses.cbreak()
    screen.keypad(True)
    i = 0
    try:
        while True:
            key = screen.getch()
            if key == ord('q'):
                break

            screen.addstr('{: 4}'.format(i))

            if key == curses.KEY_DOWN:
                stepper_direction_down()
                stepper_step()
                i -= 1
            elif key == curses.KEY_UP:
                stepper_direction_up()
                stepper_step()
                i += 1

    finally:
        curses.nocbreak()
        screen.keypad(0)
        curses.echo()
        curses.endwin()
