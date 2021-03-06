################################################################################
# Zerynth Secure Sockets
#
# Created by Zerynth Team 2015 CC
# Authors: G. Baldi, D. Mazzei
################################################################################

import streams
import json
# import the gsm interface
from wireless import gsm
# import the http module
import requests
import ssl

from quectel.bg96 import bg96 as bg96


streams.serial()

try:
    print("Initializing UG96...")
    # init the ug96
    # pins and serial port must be set according to your setup
    bg96.init(SERIAL2, D42, D42, D42, D42, D34,0,1,1)
    bg96.startup()


    # change APN name as needed
    print("Establishing Link...")
    gsm.attach("YOUR-APN-HERE")

    # let's try to connect to https://www.howsmyssl.com/a/check to get some info
    # on the SSL/TLS connection

    # retrieve the CA certificate used to sign the howsmyssl.com certificate
    cacert = __lookup(SSL_CACERT_DST_ROOT_CA_X3)

    # create a SSL context to require server certificate verification
    ctx = ssl.create_ssl_context(cacert=cacert,options=ssl.CERT_REQUIRED|ssl.SERVER_AUTH)
    # NOTE: if the underlying SSL driver does not support certificate validation
    #       uncomment the following line!
    # ctx = None


    for i in range(3):
        try:
            print("Trying to connect...")
            url="https://www.howsmyssl.com/a/check"
            # url resolution and http protocol handling are hidden inside the requests module
            user_agent = {"User-Agent": "curl/7.53.1", "Accept": "*/*" }
            # pass the ssl context together with the request
            response = requests.get(url,headers=user_agent,ctx=ctx)
            # if we get here, there has been no exception, exit the loop
            break
        except Exception as e:
            print(e)


    try:
        # check status and print the result
        if response.status==200:
            print("Success!!")
            print("-------------")
            # it's time to parse the json response
            js = json.loads(response.content)
            # super easy!
            for k,v in js.items():
                if k=="given_cipher_suites":
                    print("Supported Ciphers")
                    for cipher in v:
                        print(cipher)
                    print("-----")
                else:
                    print(k,"::",v)
            print("-------------")
    except Exception as e:
        print("ooops, something very wrong! :(",e)
except Exception as e:
    print("oops, exception!",e)
