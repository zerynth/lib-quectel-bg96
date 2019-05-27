"""
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

Listening sockets for TCP and UDP protocols are not implemented due to the nature of GSM networks. 
Moreover, UDP sockets must be connected or bind explicitly in the code to select which kind of function to perform (send vs sendto and recv vs recvfrom).
NB IoT support is not ready yet.

The communication with BG96 is performed via UART without hardware flow control at 115200 baud.

This module provides the :samp:`bg96Exception` to signal errors related to the hardware initialization and management.

   """
import streams


new_exception(bg96Exception, Exception)


def init(serial,dtr,rts,poweron,reset,status,pe=None):
    """
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
    *pe* should be an instance of the :ref:`GPIO module <stdlib_gpio>`.

    """
    _init(serial,dtr,rts,poweron,reset,status,__nameof(bg96Exception))
    if pe:
        _startup_py(serial,dtr,rts,poweron,reset,status,pe)
    else:
        _startup(0)
    __builtins__.__default_net["gsm"] = __module__
    __builtins__.__default_net["ssl"] = __module__
    __builtins__.__default_net["sock"][0] = __module__ #AF_INET

@c_native("_bg96_init",["csrc/bg96.c","csrc/bg96_ifc.c"])
def _init(serial,dtr,rst,poweron,reset,status,exc):
    pass

@c_native("_bg96_startup",["csrc/bg96.c"])
def _startup(skip_poweron):
    pass

def _startup_py(serial,dtr,rts,poweron,reset,status,pe):
    # print("Setting Pins...");
    pe.mode(status,INPUT);
    pe.mode(poweron,OUTPUT_PUSHPULL);
    pe.mode(reset,OUTPUT_PUSHPULL);

    # print("Powering off...");
    if not pe.get(status):
        # already powered down
        # print("Already down")
        sleep(3000)
    else:
        # print("Manually down")
        pe.set(poweron,1)
        sleep(800)
        pe.set(poweron,0)
        sleep(800)
        for i in range(50):
            if not pe.get(status):
                # print("STA!")
                break
            else:
                # print("!STA")
                sleep(100)
        sleep(1000)


    # print("Powering on...")
    pe.set(poweron,1)
    sleep(800)
    pe.set(poweron, 0)
    sleep(800)
    pe.set(poweron, 1)
    for i in range(50):
        if pe.get(status):
            # print("STA!")
            break
        else:
            # print("!STA")
            sleep(100)
    if not pe.get(status):
        # can't power up :(
        # print("Can't power up!")
        raise HardwareInitializationError

    _startup(1)
    # # Wait for RDY
    # print("Wait for RDY")
    # for i in range(100):
    #     if not ss.available():
    #         sleep(150)
    #     r = ss.readline()
    #     if r.startswith("RDY"):
    #         break
    #     else:
    #         print("GOT",r)
    # else:
    #     raise HardwareInitializationError



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

@c_native("_bg96_set_rat",[])
def _set_rat(rat,bands):
    pass

def set_rat(rat,bands):
    # prepare the bands according to QCFG manual entry
    # band definition here: https://en.wikipedia.org/wiki/LTE_frequency_bands
    pbands=0
    if rat==0:
        # gsm bands
        for b in bands:
            if b in [0,1,2,3]:
                pbands= pbands| (1<<b)
        _set_rat(rat,pbands)
    else:
        # lte bands
        for i,b in enumerate(bands):
            if b in [1,2,3,4,5,8,12,13,18,19,20,26,28]:
                pbands = pbands | (1<<i)
        _set_rat(rat,pbands)


@native_c("_bg96_socket_bind",[])
def bind(sock,addr):
    pass

def listen(sock,maxlog=2):
    raise UnsupportedError

def accept(sock):
    raise UnsupportedError


@native_c("_bg96_resolve",[])
def gethostbyname(hostname):
    pass


@native_c("_bg96_socket_create",[])
def socket(family,type,proto):
    pass

def setsockopt(sock,level,optname,value):
    pass

@native_c("_bg96_socket_connect",[])
def connect(sock,addr):
    pass

@native_c("_bg96_socket_close",[])
def close(sock):
    pass


@native_c("_bg96_socket_sendto",[])
def sendto(sock,buf,addr,flags=0):
    pass

@native_c("_bg96_socket_send",[])
def send(sock,buf,flags=0):
    pass

def sendall(sock,buf,flags=0):
    send(sock,buf,flags)

@native_c("_bg96_socket_recv_into",[])
def recv_into(sock,buf,bufsize,flags=0,ofs=0):
    pass

@native_c("_bg96_socket_recvfrom_into",[])
def recvfrom_into(sock,buf,bufsize,flags=0):
    pass

@native_c("_bg96_secure_socket",[],[])
def secure_socket(family, type, proto, ctx):
    pass

@native_c("_bg96_socket_select",[])
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
def gnss_init():
    """
----
GNSS
----

The BG96 has an integrated GNSS that can be activated and queried for location fixes, regardless of the network status.

.. function:: gnss_init()
   
    Initializes the GNSS subsystem.

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
   
    Return a tuple of 8 elements:

        * latitude in decimal format (-89.9999 - 89.9999)
        * longitude in decimal format (-179.9999 - 179.9999)
        * altitude in meters
        * speed in Km/h
        * course over ground as degrees from true north
        * horizontal dilution of precision (0.5 - 99.9)
        * number of satellites for this fix
        * UTC time as a tuple (yyyy,MM,dd,hh,mm,ss)

    The function return None if a fix can't be obtained.

    """
    pass


