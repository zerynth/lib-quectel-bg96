################################################################################
# BG96 GNSS System
#
# Created by Zerynth Team 2019 CC
# Authors: D. Mazzei, G. Baldi
################################################################################

import streams
from quectel.bg96 import bg96

streams.serial()


try:
    # let's init the BG96 driver
    # Change serial port and pins according to your setup!
    bg96.init(SERIAL2, D27, D26, D21, D12, D25)

    # Init GNSS
    print("Initializing GNSS...")
    bg96.gnss_init()
    while True:
        fix = bg96.fix()
        if not fix:
            print("No fix yet...")
        else:
            print(fix)
        sleep(1000)
except Exception as e:
    print(e)

while True:
    print(".")
    sleep(1000)

