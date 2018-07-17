#!/usr/bin/env python3

import time
import statistics
import sqlite3
from olsndot import Olsndot, Driver
from datetime import datetime

uut1 = Olsndot(0x23420001)
uut2 = Olsndot(0x23420002)
d = Driver('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0', devices=[uut1, uut2])

print('uut 1:', uut1.fetch_status())
print('uut 2:', uut2.fetch_status())

