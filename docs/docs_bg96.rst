.. module:: bg96

***********
BG96 Module
***********

This module implements the Zerynth driver for the Quectel BG96 gsm/gprs LTE Cat M1 and NB1 chip (`Product page <https://www.quectel.com/product/bg96.htm>`_).

The driver must be used together with the standard library :ref:`GSM Module <stdlib_gsm>`.

The following functionalities are implemented:

    * attach/detach from gprs network
    * retrieve and set available operators
    * retrieve signal strength
    * retrieve network and device info
    * socket abstraction
    * secure sockets
    * GNSS positioning
    * RTC clock

Listening sockets for TCP and UDP protocols are not implemented due to the nature of GSM networks. NB IoT support is not ready yet.

The communication with BG96 is performed via UART without hardware flow control.

This module provides the :samp:`bg96Exception` to signal errors related to the hardware initialization and management.

   
.. function:: init(serial,dtr,rts,poweron,reset,status,pe=None)

    Initialize the BG96 device given the following parameters:

    * *serial*, the serial port connected to the BG96 (:samp;`SERIAL1`,:samp:`SERIAL2`, etc..)
    * *dtr*, the DTR pin of BG96
    * *rts*, the RTS pin of BG96
    * *poweron*, the power up pin of BG96
    * *reset*, the reset pin of BG96
    * *status*, the status pin of BG96
    * *pe*, a port expander implementation

    If *pe* is not None, the BG96 initialization is performed using *pe* to control the pins.
    *pe* should be an instance of the :ref:`GPIO odle <stdlib_gpio>`.

    
------------
Network Time
------------

The BG96 has an internal Real Time Clock that is automatically synchronized with the Network Time.
The current time can be retrieved with the following function:

.. function:: rtc()
    
    Return a tuple of seven elements:

        * current year
        * current month (1-12)
        * current day (1-31)
        * current hour (0-23)
        * current minute (0-59)
        * current second (0-59)
        * current timezone in minutes away from GMT 0

    The returned time is always UTC time with a timezone indication.

    
----
GNSS
----

The BG96 has an integrated GNSS that can be activated and queried for location fixes, regardless of the network status.

.. function:: gnss_init()
   
    Initializes the GNSS subsystem.

    
.. function:: gnss_done()
   
    Shutdown the GNSS subsystem.

    
.. function:: gnss_fix()
   
    Return a tuple of 8 elements:

        * latitude in decimal format
        * longitude in decimal format
        * altitude in meters
        * speed in Km/h
        * course over ground as degrees from true north
        * horizontal dilution of precision (0.5 - 99.9)
        * number of satellite for this fix
        * UTC time as a tuple (yyyy,MM,dd,hh,mm,ss)

    The function return None if a fix can't be obtained.

    
