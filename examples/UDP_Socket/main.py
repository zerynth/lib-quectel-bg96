################################################################################
# Zerynth UDP Socket
#
# Created by Zerynth Team 2015 CC
# Authors: G. Baldi, D. Mazzei
################################################################################

import streams
import socket
# import the gsm interface
from wireless import gsm
from quectel.ug96 import ug96 as ug96

# For this example to work, you need an UDP server somewhere on the public
# internet. You can run this example (https://gist.github.com/Manouchehri/67b53ecdc767919dddf3ec4ea8098b20)
# on your cloud instance on the port you prefer for the sake of this test

streams.serial()

# specify here the IP and port of your UDP server
server_ip = "0.0.0.0"
server_port = 7778

try:
    print("Initializing UG96...")
    # init the ug96
    # pins and serial port must be set according to your setup
    ug96.init(SERIAL3,D12,D13,D67,D60,D37,D38,0)


    # use the wifi interface to link to the Access Point
    # change network name, security and password as needed
    print("Establishing Link...")
    gsm.attach("your-apn-name")


    print("Trying UDP socket in connect mode")
    # Let's open an udp socket with connect.
    # the socket will then be used with send and recv methods
    # without specifying the receiver address.
    # The socket will be able to send only to the address
    # specified in the connect method
    sock= socket.socket(type=socket.SOCK_DGRAM,proto=socket.IPPROTO_UDP)
    sock.connect((server_ip,server_port))
    for i in range(5):
        sock.send("Hello\n")
        sleep(1000)

    sock.close()

    print("Trying UDP socket in bind mode")
    # Let's open an udp socket and configure with bind.
    # the socket will then be used with sendto and recvfrom methods
    # specifying the receiver address.
    # The socket will be able to send to any ip address
    sock= socket.socket(type=socket.SOCK_DGRAM,proto=socket.IPPROTO_UDP)
    # open the udp socket on port 5678 on the public facing ip
    # Note: you won't necessarily see the same origin port on the
    # UDP server since GSM networks are usually behind a NAT
    sock.bind(("0.0.0.0",5678))
    for i in range(5):
        sock.sendto("Hello\n",(server_ip,server_port))
        sleep(1000)

except Exception as e:
    print("oops, exception!",e)

while True:
    print(".")
    sleep(1000)


