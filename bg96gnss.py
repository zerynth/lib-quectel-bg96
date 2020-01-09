"""
.. module:: bg96gnss

**********
BG96_GNSS Module
**********

This module implements the Zerynth driver for the Quectel BG96 GNSS functionality
on its dedicated UART port (`Product page <https://www.quectel.com/product/bg96gnss.htm>`_).


The following functionalities are implemented:

    * retrieve the current location fix if present
    * retrieve the current UTC time

The driver starts a background thread continuously tracking the last available location fix.
The frequency of fixes can be customized.
The driver support serial mode only.

Location fixes are obtained by parsing NMEA sentences of type RMC and GGA.
Obtaining a fix or UTC time are thread safe operations.

    """

import streams
import threading
from quectel.bg96 import bg96
from quectel.nmea import nmea

class BG96_GNSS(nmea.NMEA_Receiver):
    """
.. class:: BG96_GNSS(ifc, baud=9600)

    Create an instance of the BG96_GNSS class.

    :param ifc: serial interface to use (for example :samp:`SERIAL1`, :samp:`SERIAL2`, etc...)
    :param baud: serial port baudrate

    Example: ::

        from quectel.bg96gnss import bg96gnss

        ...

        gnss = bg96gnss.BG96_GNSS(SERIAL1)
        gnss.start()
        mpl.init()
        alt = mpl.get_alt()
        pres = mpl.get_pres()

    """

    def __init__(self,ifc,baud=9600,antpower=None,antpower_on=1):
        self.ifc = ifc
        self.baud = baud
        self.running = False
        self.talking = False
        self.drv = None
        self.th = None
        self.fixrate = 1
        self.antpin = antpower
        self.antval = antpower_on
        nmea.NMEA_Receiver.__init__(self)
        if self.antpin is not None:
            pinMode(self.antpin, OUTPUT)
            digitalWrite(self.antpin, HIGH^ self.antval)

    def start(self):
        """
.. method:: start()

        Start the BG96 GNSS and the receiver thread.

        :returns: *True* if receiver thread has been started, *False* if already active.

        """
        if self.th:
            return False
        
        bg96.startup(_from_gnss=True)
        if self.antpin is not None:
            digitalWrite(self.antpin, self.antval) # power on antenna

        self.enable(True)
        self.running = True
        self.talking = True
        self.th = thread(self._run)
        bg96.gnss_init(fix_rate=self.fixrate,use_uart=1)
        sleep(1000)
        return True

    def stop(self):
        """
.. method:: stop()

        Stop the BG96 GNSS and terminates the receiver thread.
        It can be restarted by calling :ref:`start`.

        :returns: *True* if receiver thread has been stopped, *False* if already inactive.

        """
        if not self.running:
            return False

        if bg96._gnss_active: # prevent exception if modem was forced off
            bg96.gnss_done()
        self.enable(False)
        self.running = False
        self.talking = False
        if self.antpin is not None:
            digitalWrite(self.antpin, HIGH^ self.antval) # power off antenna
        sleep(1000)
        
        bg96.shutdown(_from_gnss=True)
        return True

    def pause(self):
        """
.. method:: pause()

        Pause the BG96 GNSS by putting it into standby mode.
        It can be restarted by calling :ref:`resume`.

        """
        if not self.running:
            raise RuntimeError
        bg96.gnss_done()
        self.enable(False)
        self.talking = False
        if self.antpin is not None:
            digitalWrite(self.antpin, HIGH^ self.antval) # power off antenna

    def resume(self):
        """
.. method:: resume()

        Wake up the BG96_GNSS from standby mode.

        """
        if not self.running:
            raise RuntimeError
        self.enable(True)
        if self.antpin is not None:
            digitalWrite(self.antpin, self.antval) # power on antenna
        self.talking = True
        bg96.gnss_init(fix_rate=self.fixrate,use_uart=1)

    def set_rate(self,rate=1000):
        """
.. method:: set_rate(rate=1000)

        Set the frequency for location fix (100-10000 milliseconds is the available range).

        """
        if self.talking:
            bg96.gnss_done()

        self.fixrate = rate // 1000

        if self.talking:
            bg96.gnss_init(fix_rate=self.fixrate,use_uart=1)

    ##################### Private

    def _run(self):
        """
----
NMEA
----

    Additional methods from the base class :any:`nmea.NMEA_Receiver`.

.. method:: fix()

        Return the current fix or *None* if not available.
        A fix is a tuple with the following elements:

            * latitude in decimal format (-89.9999 - 89.9999)
            * longitude in decimal format (-179.9999 - 179.9999)
            * altitude in meters
            * speed in Km/h
            * course over ground as degrees from true north
            * number of satellites for this fix
            * horizontal dilution of precision (0.5 - 99.9)
            * vertical dilution of precision (0.5 - 99.9)
            * positional dilution of precision (0.5 - 99.9)
            * UTC time as a tuple (yyyy,MM,dd,hh,mm,ss,microseconds)

.. method:: has_fix()
    
        Return *True* if a fix is available

.. method:: utc()

        Return the current UTC time or *None* if not available.
        A UTC time is a tuple of (yyyy,MM,dd,hh,mm,ss,microseconds).

        UTC time can be wrong if no fix has ever been obtained.

.. method:: has_utc()
    
        Return *True* if a UTC time is available

        """
        buffer = bytearray(256)
        self.drv = streams.serial(self.ifc,baud=self.baud,set_default=False)

        while self.running:
            try:
                chs = nmea.readline(self.ifc,buffer)
                if self.debug:
                    self.print_d(buffer[:buffer.find(b'\0')])
                if not self.talking:
                    continue
                if chs>=1:
                    self.parse(buffer, chs)
                else:
                    self.print_d("BG96_GNSS check",chs)
            except Exception as e:
                if self.talking:
                    self.print_d("BG96_GNSS loop", e)

        self.drv.close()
        self.th = None
