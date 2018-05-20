#!/usr/bin/env python3

import time
import statistics
import sqlite3
from olsndot import Olsndot, Driver
from datetime import datetime

from pyBusPirateLite import Buspirate

uut = Olsndot(0xDEBE10BB)
d = Driver('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0', devices=[uut])


