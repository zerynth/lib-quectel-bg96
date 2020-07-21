"""
.. module:: bg96

***********
BG96 Module
***********

This module implements the Zerynth driver for the Quectel BG96 LTE Cat-M1, LTE Cat-NB1 and EGPRS modem (`Product page <https://www.quectel.com/product/bg96.htm>`_).

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

Listening sockets for TCP and UDP protocols are not implemented due to the nature of GSM networks. 
Moreover, UDP sockets must be connected or bound explicitly in the code to select which kind of function to perform (send vs sendto and recv vs recvfrom).
NB IoT support is not ready yet.

The communication with BG96 is performed via UART without hardware flow control at 115200 baud.

This module provides the :samp:`bg96Exception` to signal errors related to the hardware initialization and management.

   """
import gpio

new_exception(bg96Exception, Exception)
_reset_pin=None
_reset_on=None
_power_pin=None
_power_on=None
_status_pin=None
_status_on=None

_gnss_active=False
_modem_active=False

def init(serial,dtr,rts,power,reset,status,power_on=LOW,reset_on=LOW,status_on=HIGH):
    """
.. function:: init(serial,dtr,rts,power,reset,status,power_on=LOW,reset_on=LOW,status_on=HIGH)

    Initialize the BG96 device given the following parameters:

    * *serial*, the serial port connected to the BG96 (:samp:`SERIAL1`, :samp:`SERIAL2`, etc...)
    * *dtr*, the DTR pin of BG96
    * *rts*, the RTS pin of BG96
    * *power*, the power up pin of BG96
    * *reset*, the reset pin of BG96
    * *status*, the status pin of BG96
    * *power_on*, the active level of the power up pin
    * *reset_on*, the active level of the reset pin
    * *status_on*, the value of status pin indicating successful power on (can be zero in some pcb designs)

    """
    global _reset_pin, _reset_on, _power_pin, _power_on, _status_pin, _status_on
    _reset_pin=reset
    _reset_on=reset_on
    _power_pin=power
    _power_on=power_on
    _status_pin=status
    _status_on=status_on

    # print("Setting Pins...");
    gpio.mode(_status_pin, INPUT_PULLDOWN if _status_on else INPUT_PULLUP)
    gpio.mode(_reset_pin, OUTPUT_PUSHPULL)
    gpio.set(_reset_pin, HIGH^ _reset_on)
    gpio.mode(_power_pin, OUTPUT_PUSHPULL)
    gpio.set(_power_pin, HIGH^ _power_on)

    _init(serial,dtr,rts,__nameof(bg96Exception))
    __builtins__.__default_net["gsm"] = __module__
    __builtins__.__default_net["ssl"] = __module__
    __builtins__.__default_net["sock"][0] = __module__ #AF_INET

    shutdown(True)

@c_native("_bg96_init",[ 
        "csrc/bg96.c",
        "csrc/bg96_ifc.c",
        "#csrc/misc/zstdlib.c",
        "#csrc/misc/snprintf.c",
        "#csrc/zsockets/*",
        "#csrc/hwcrypto/*",
        #-if ZERYNTH_SSL
        ##-if !HAS_BUILTIN_MBEDTLS
        "#csrc/tls/mbedtls/library/*",
        ##-endif
        #-endif
    ],
    [
        "VHAL_WIFI"
    ],
    [
        "-I#csrc/zsockets",
        "-I#csrc/misc",
        "-I#csrc/hwcrypto",
        #-if ZERYNTH_SSL
        ##-if !HAS_BUILTIN_MBEDTLS
        "-I#csrc/tls/mbedtls/include"
        ##-endif
        #-endif
    ])
def _init(serial,dtr,rst,exc):
    pass

@c_native("_bg96_shutdown",[])
def _shutdown(only_modem):
    pass

@c_native("_bg96_startup",[])
def _startup(without_modem):
    pass

@c_native("_bg96_bypass",[])
def bypass(mode):
    """
.. function:: bypass(mode)

    Bypass the modem driver to use the serial port directly. It has one parameter:

    * *mode*, can be *1* (non-zero) to enter bypass mode, or *0* (zero) to exit.
    
    """
    pass

def shutdown(forced=False,_from_gnss=False):
    """
.. function:: shutdown(forced=False)

    Power off the module by pulsing the power pin (clean power-down).

    If *forced* is given, use the reset pin (faster, do not detach from network).
    """
    # keep running for GNSS
    global _gnss_active,_modem_active
    if forced:
        _modem_active = False
        _gnss_active = False
    elif _from_gnss:
        _gnss_active = False
    else:
        _modem_active = False
    # only call if modem is inactive
    if not _modem_active:
        if _shutdown(not _modem_active and _gnss_active):
            # normal shutdown attempted
            for i in range(30):
                if gpio.get(_status_pin)!=_status_on:
                    # print("!STA")
                    break
                # print("STA!")
                sleep(100)
    # full hardware power off if both inactive
    if _modem_active or _gnss_active:
        return
    # print("Powering off...")
    if gpio.get(_status_pin)==_status_on and forced:
        gpio.set(_reset_pin, HIGH^ _reset_on)
        sleep(200)
        gpio.set(_reset_pin, _reset_on)
        sleep(300)
        gpio.set(_reset_pin, HIGH^ _reset_on)

        for i in range(55):
            if gpio.get(_status_pin)==_status_on:
                # print("f STA!")
                break
            # print("f !STA")
            sleep(100)
    
    if gpio.get(_status_pin)==_status_on:
        gpio.set(_power_pin, HIGH^ _power_on)
        sleep(500)
        gpio.set(_power_pin, _power_on)
        sleep(700)
        gpio.set(_power_pin, HIGH^ _power_on)

    for i in range(30):
        if gpio.get(_status_pin)!=_status_on:
            # print("!STA")
            break
        # print("STA!")
        sleep(100)
    else:
        raise HardwareInitializationError
    sleep(500)

def startup(_from_gnss=False):
    """
.. function:: startup()

    Power on the module by pulsing the power pin. 
    """
    # print("Powering on...")
    if gpio.get(_status_pin)!=_status_on:
        gpio.set(_power_pin, HIGH^ _power_on)
        sleep(500)
        gpio.set(_power_pin, _power_on)
        sleep(600)
        gpio.set(_power_pin, HIGH^ _power_on)

    for i in range(100):
        if gpio.get(_status_pin)==_status_on:
            # print("STA!")
            break
        # print("!STA")
        sleep(100)
    else:
        raise HardwareInitializationError
    # turn off modem if only for GNSS
    global _gnss_active,_modem_active
    if not _modem_active:
        _startup(_from_gnss)
    if _from_gnss:
        _gnss_active = True
    else:
        _modem_active = True

@c_native("_bg96_attach",[])
def attach(apn,username,password,authmode,timeout):
    pass

@c_native("_bg96_detach",[])
def detach():
    pass

@c_native("_bg96_network_info",[])
def network_info():
    pass

@c_native("_bg96_mobile_info",[])
def mobile_info():
    pass

@c_native("_bg96_link_info",[])
def link_info():
    pass

@c_native("_bg96_operators",[])
def operators():
    pass

@c_native("_bg96_set_operator",[])
def set_operator(opname):
    pass

# @c_native("_bg96_set_rat",[])
# def _set_rat(rat,bands):
#     pass

# def set_rat(rat,bands):
#     # prepare the bands according to QCFG manual entry
#     # band definition here: https://en.wikipedia.org/wiki/LTE_frequency_bands
#     pbands=0
#     if rat==0:
#         # gsm bands
#         for b in bands:
#             if b in [0,1,2,3]:
#                 pbands= pbands| (1<<b)
#         _set_rat(rat,pbands)
#     else:
#         # lte bands
#         for i,b in enumerate(bands):
#             if b in [1,2,3,4,5,8,12,13,18,19,20,26,28]:
#                 pbands = pbands | (1<<i)
#         _set_rat(rat,pbands)


@native_c("py_net_bind",[])
def bind(sock,addr):
    pass

def listen(sock,maxlog=2):
    raise UnsupportedError

def accept(sock):
    raise UnsupportedError


@native_c("_bg96_resolve",[])
def gethostbyname(hostname):
    pass


@native_c("py_net_socket",[])
def socket(family,type,proto):
    pass

def setsockopt(sock,level,optname,value):
    pass

@native_c("py_net_connect",[])
def connect(sock,addr):
    pass

@native_c("py_net_close",[])
def close(sock):
    pass


@native_c("py_net_sendto",[])
def sendto(sock,buf,addr,flags=0):
    pass

@native_c("py_net_send",[])
def send(sock,buf,flags=0):
    pass

def sendall(sock,buf,flags=0):
    send(sock,buf,flags)

@native_c("py_net_recv_into",[])
def recv_into(sock,buf,bufsize,flags=0,ofs=0):
    pass

@native_c("py_net_recvfrom_into",[])
def recvfrom_into(sock,buf,bufsize,flags=0,ofs=0):
    pass

@native_c("py_secure_socket",[],[])
def secure_socket(family, type, proto, ctx):
    pass

@native_c("py_net_select",[])
def select(rlist,wist,xlist,timeout):
    pass

@native_c("_bg96_rtc",[])
def rtc():
    """
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

    """
    pass

@c_native("_bg96_rssi",[])
def rssi():
    pass


############# GNSS system

@c_native("_bg96_gnss_init",[])
def gnss_init(fix_rate=1,use_uart=0):
    """
----
GNSS
----

The BG96 has an integrated GNSS that can be activated and queried for location fixes, regardless of the network status.

A separate module :any:`bg96gnss` is also provided to parse NMEA senteces directly from the BG96 dedicated serial port.
When using the :any:`BG96_GNSS` class, do not also call the following methods to avoid conflicts.

.. function:: gnss_init(fix_rate=1,use_uart=0)
   
    Initializes the GNSS subsystem, given the following parameters:

    * *fix_rate*, configure GNSS fix or NMEA output rate in seconds
    * *use_uart*, use the secondary serial port (UART3) of the BG96 to output NMEA sentences

    """
    pass

@c_native("_bg96_gnss_done",[])
def gnss_done():
    """
.. function:: gnss_done()
   
    Shutdown the GNSS subsystem.

    """
    pass

@c_native("_bg96_gnss_fix",[])
def fix():
    """
.. function:: fix()
   
    Return a tuple of 10 elements:

        * latitude in decimal format (-89.9999 - 89.9999)
        * longitude in decimal format (-179.9999 - 179.9999)
        * altitude in meters
        * speed in Km/h
        * course over ground as degrees from true north
        * number of satellites for this fix
        * horizontal dilution of precision (0.5 - 99.9)
        * Not supported
        * Not supported
        * UTC time as a tuple (yyyy,MM,dd,hh,mm,ss)

    The function return None if a fix can't be obtained.

    """
    pass

@c_native("_bg96_sms_send",[])
def send_sms(num,txt):
    pass

@c_native("_bg96_sms_delete",[])
def delete_sms(index):
    pass

@c_native("_bg96_sms_list",[])
def list_sms(unread,maxsms,offset):
    pass

@c_native("_bg96_sms_pending",[])
def pending_sms():
    pass

@c_native("_bg96_sms_get_scsa",[])
def get_smsc():
    pass

@c_native("_bg96_sms_set_scsa",[])
def set_smsc(scsa):
    pass

