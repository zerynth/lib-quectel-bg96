#include "zerynth.h"
#include "bg96.h"

//a reference to a Python exception to be returned on error (bg96Exception)
int32_t bg96exc;

#if 0
#define printf(...) vbl_printf_stdout(__VA_ARGS__)
#define print_buffer(bf, ln) 	for(uint8_t i = 0; i < ln; i++) { \
					printf("%c", bf[i]); \
				} \
				printf("\n"); 
#else
#define printf(...)
#define print_buffer(bf, ln)
#endif

///////// CNATIVES
// The following functions are callable from Python.
// Functions starting with "_" are utility functions called by CNatives


SocketAPIPointers bg96_api;
uint8_t *_urats;
uint8_t _urats_len[] = {3, 4, 3, 10, 11};
uint8_t _urats_pos[] = {0, 3, 7, 10, 20};
/**
 * @brief _bg96_init calls _gs_init, _gs_poweron and _gs_config0
 *
 * As last parameter, requires an integer saved to global bg96exc, representing the name assigned to bg96Exception
 * so that it can be raised by returning bg96exc. If modules initialization is successful, the next step is calling startup
 *
 */
C_NATIVE(_bg96_init){
    NATIVE_UNWARN();
    int32_t serial;
    int32_t rts;
    int32_t dtr;
    int32_t err = ERR_OK;
    int32_t exc;

    if (parse_py_args("iiii", nargs, args, &serial, &dtr, &rts, &exc) != 4) return ERR_TYPE_EXC;

    bg96exc = exc;

    *res = MAKE_NONE();


    RELEASE_GIL();
    _gs_init();
    gs.serial = serial&0xff;
    gs.rx = _vm_serial_pins[gs.serial].rxpin;
    gs.tx = _vm_serial_pins[gs.serial].txpin;
    gs.dtr = dtr;
    gs.rts = rts;

    ACQUIRE_GIL();
    bg96_api.socket = bg96_gzsock_socket;
    bg96_api.connect = bg96_gzsock_connect;
    bg96_api.setsockopt = bg96_gzsock_setsockopt;
    bg96_api.getsockopt = bg96_gzsock_getsockopt;
    bg96_api.send = bg96_gzsock_send;
    bg96_api.sendto = bg96_gzsock_sendto;
    bg96_api.write = bg96_gzsock_write;
    bg96_api.recv = bg96_gzsock_recv;
    bg96_api.recvfrom = bg96_gzsock_recvfrom;
    bg96_api.read = bg96_gzsock_read;
    bg96_api.close = bg96_gzsock_close;
    bg96_api.shutdown = bg96_gzsock_shutdown;
    bg96_api.bind = bg96_gzsock_bind;
    bg96_api.accept = NULL;
    bg96_api.listen = NULL;
    bg96_api.select = bg96_gzsock_select;
    bg96_api.fcntl = bg96_gzsock_fcntl;
    bg96_api.ioctl = NULL;
    bg96_api.getaddrinfo = bg96_gzsock_getaddrinfo;
    bg96_api.freeaddrinfo = bg96_gzsock_freeaddrinfo;
    bg96_api.inet_addr = NULL;
    bg96_api.inet_ntoa = NULL;

    printf("After init\n");
    gzsock_init(&bg96_api);
    printf("After gzsock_init\n");
    _urats = gc_malloc(3 + 4 + 3 + 10 + 11);
    memcpy(_urats, "GSMGPRSLTELTE Cat M1LTE Cat NB1", 31);
    return err;
}

/**
 * @brief Setup modem serial port, AT configuration and start modem thread
 *
 * Optionally disable modem (to save power when only GNSS in use)
 */
C_NATIVE(_bg96_startup){
    NATIVE_UNWARN();
    int32_t err = ERR_OK;
    int32_t without_modem;
    *res = MAKE_NONE();

    if (parse_py_args("i", nargs, args, &without_modem) != 1) return ERR_TYPE_EXC;

    RELEASE_GIL();
    vosSemWait(gs.slotlock);

    if (_gs_stop() != 0)
        err = ERR_HARDWARE_INITIALIZATION_ERROR;
    else
    if (vhalSerialInit(gs.serial, 115200, SERIAL_CFG(SERIAL_PARITY_NONE,SERIAL_STOP_ONE, SERIAL_BITS_8,0,0), gs.rx, gs.tx) != 0)
        err = ERR_HARDWARE_INITIALIZATION_ERROR;
    else
    if (!_gs_config0(without_modem))
        err = ERR_HARDWARE_INITIALIZATION_ERROR;
    else {
        if (gs.thread==NULL){
            //let's start modem thread (if not already started)
            printf("Starting modem thread with size %i\n",VM_DEFAULT_THREAD_SIZE);
            gs.thread = vosThCreate(VM_DEFAULT_THREAD_SIZE,VOS_PRIO_NORMAL,_gs_loop,NULL,NULL);
            vosThResume(gs.thread);
            vosThSleep(TIME_U(1000,MILLIS)); // let modem thread have a chance to start
        }
    }
    // reset driver status (assuming modem has restarted)
    gs.attached = 0;
    gs.registered = 0;
    gs.gsm_status = 0;
    gs.gprs_status = 0;
    gs.eps_status = 0;
    gs.registration_status_time = (uint32_t)(vosMillis() / 1000);

    // start loop and wait
    if (_gs_start() != 0)
        err = ERR_HARDWARE_INITIALIZATION_ERROR;

    vosSemSignal(gs.slotlock);
    ACQUIRE_GIL();
    return err;
}

/**
 * @brief Stop modem thread and close serial port
 *
 * Optionally disable modem (to save power when only GNSS in use)
 */
C_NATIVE(_bg96_shutdown){
    NATIVE_UNWARN();
    int32_t err = ERR_OK;
    int32_t only_modem;
    *res = MAKE_NONE();
    
    if (parse_py_args("i", nargs, args, &only_modem) != 1) return ERR_TYPE_EXC;
    printf("%s only_modem=%d\n", __FUNCTION__, only_modem);
    
    RELEASE_GIL();
    if (only_modem) {
        if (!_gs_modem_functionality(0))
            err = ERR_HARDWARE_INITIALIZATION_ERROR;
    }
    else {
        vosSemWait(gs.slotlock);

        if (_gs_stop() != 0)
            err = ERR_HARDWARE_INITIALIZATION_ERROR;
        
        // attempt normal shutdown
        vhalSerialInit(gs.serial, 115200, SERIAL_CFG(SERIAL_PARITY_NONE,SERIAL_STOP_ONE, SERIAL_BITS_8,0,0), gs.rx, gs.tx);
        // check alive
        vhalSerialWrite(gs.serial, "ATE0\r\n", 6);
        if (_gs_wait_for_ok(500)) {
            //enter minimal functionality
            vhalSerialWrite(gs.serial, "AT+CFUN=0\r\n", 11);
            _gs_wait_for_ok(15000);
            //power down
            vhalSerialWrite(gs.serial, "AT+QPOWD\r\n", 10);
            *res = PSMALLINT_NEW(1);
        }
        vhalSerialDone(gs.serial);

        vosSemSignal(gs.slotlock);
    }
    ACQUIRE_GIL();
    return err;
}

/**
 * @brief Stop/restart modem thread
 *
 * Give direct access to modem serial port
 */
C_NATIVE(_bg96_bypass){
    NATIVE_UNWARN();
    int32_t mode;
    int32_t err=ERR_OK;

    if(parse_py_args("i",nargs,args,&mode)!=1) return ERR_TYPE_EXC;

    *res = MAKE_NONE();
    if (mode) {
        vosSemWait(gs.slotlock);
        if (_gs_stop() != 0)
            err = ERR_HARDWARE_INITIALIZATION_ERROR;
    }
    else {
        if (_gs_start() != 0)
            err = ERR_HARDWARE_INITIALIZATION_ERROR;
        vosSemSignal(gs.slotlock);
    }
    return err;
}


/**
 * @brief _bg96_detach removes the link with the APN while keeping connected to the GSM network
 *
 *
 */
C_NATIVE(_bg96_detach){
    NATIVE_UNWARN();
    int err = ERR_OK;
    *res = MAKE_NONE();
    RELEASE_GIL();

    if(!_gs_control_psd(0))
        err = bg96exc;

    ACQUIRE_GIL();
    return err;
}


/**
 * @brief _bg96_attach tries to link to the given APN
 *
 * This function can block for a very long time (up to 2 minutes) due to long timeout of used AT commands
 *
 *
 */
C_NATIVE(_bg96_attach){
    NATIVE_UNWARN();
    uint8_t *apn;
    uint32_t apn_len;
    uint8_t *user;
    uint32_t user_len;
    uint8_t *password;
    uint32_t password_len;
    uint32_t authmode;
    int32_t timeout;
    int32_t err=ERR_OK;

    if(parse_py_args("sssii",nargs,args,&apn,&apn_len,&user,&user_len,&password,&password_len,&authmode,&timeout)!=5) return ERR_TYPE_EXC;

    *res = MAKE_NONE();
    RELEASE_GIL();

    //Wait for registration
    _gs_check_network();
    while(timeout>0){
        if (gs.registered>=GS_REG_OK) break;
        vosThSleep(TIME_U(1000,MILLIS));
        timeout-=1000;
        _gs_check_network();
    }
    if (gs.registered<GS_REG_OK) {
        err = ERR_TIMEOUT_EXC;
        goto exit;
    }
    //configure PSD
    err = bg96exc;
    if(!_gs_configure_psd(apn,apn_len,user,user_len,password,password_len,authmode)) goto exit;

    //activate PSD
    if(!_gs_control_psd(1)) goto exit;

    err = ERR_OK;

exit:
    ACQUIRE_GIL();
    return err;
}



/**
 * @brief _bg96_operators retrieve the operator list and converts it to a tuple
 *
 *
 */
C_NATIVE(_bg96_operators){
    NATIVE_UNWARN();
    int i;

    RELEASE_GIL();
    i = _gs_list_operators();
    if (i){
        *res = MAKE_NONE();
        return ERR_OK;
    }
    PTuple *tpl = ptuple_new(gsopn,NULL);
    for(i=0;i<gsopn;i++){
        PTuple *tpi = ptuple_new(4,NULL);
        PTUPLE_SET_ITEM(tpi,0,PSMALLINT_NEW(gsops[i].type));
        PTUPLE_SET_ITEM(tpi,1,pstring_new(gsops[i].fmtl_l,gsops[i].fmt_long));
        PTUPLE_SET_ITEM(tpi,2,pstring_new(gsops[i].fmts_l,gsops[i].fmt_short));
        PTUPLE_SET_ITEM(tpi,3,pstring_new(gsops[i].fmtc_l,gsops[i].fmt_code));
        PTUPLE_SET_ITEM(tpl,i,tpi);
    }

    ACQUIRE_GIL();
    *res = tpl;
    return ERR_OK;
}


/**
 * @brief _bg96_set_operator try to set the current operator given its name
 *
 *
 */
C_NATIVE(_bg96_set_operator){
    NATIVE_UNWARN();
    int i;
    uint8_t *opname;
    uint32_t oplen;

    if(parse_py_args("s",nargs,args,&opname,&oplen)!=1) return ERR_TYPE_EXC;

    RELEASE_GIL();
    i = _gs_set_operator(opname,oplen);
    ACQUIRE_GIL();
    *res = MAKE_NONE();
    if (i){
        return bg96exc;
    }
    return ERR_OK;
}





/**
 * @brief _bg96_rssi return the signal strength as reported by +CIEV urc
 *
 *
 */
C_NATIVE(_bg96_rssi){
    NATIVE_UNWARN();
    int32_t rssi;

    RELEASE_GIL();
    rssi=_gs_rssi();
    ACQUIRE_GIL();

    if (rssi==99) rssi=0;
    else if (rssi<=31) rssi = -113 +2*rssi;

    *res = PSMALLINT_NEW(rssi);
    return ERR_OK;
}

/**
 * @brief strings for network types (must have the same order as bits in GS_RAT_xxx)
 */
/*const uint8_t * const _urats[] = { "","GSM","GPRS","LTE","LTE Cat M1","LTE Cat NB1" };*/
uint8_t * _urats;
#define MAX_URATS (sizeof(_urats)/sizeof(*_urats))

/**
 * @brief _bg96_network_info retrieves network information through +URAT and *CGED
 *
 *
 */
C_NATIVE(_bg96_network_info){
    NATIVE_UNWARN();
    int p0,l0,mcc,mnc;
    PString *str;
    PTuple *tpl = ptuple_new(8,NULL);
    uint8_t rats[40];
    int ratslen;

    //RAT  : URAT
    //CELL : UCELLINFO
    RELEASE_GIL();
    _gs_check_network();
    _gs_is_attached();
    if (_gs_cell_info(&mcc, &mnc) <= 0) {
        mcc = -1;
        mnc = -1;
    }
    
    // build RAT (radio access technology) string from static buffer
    // e.g. "GSM+LTE Cat M1"
    ratslen = 0;
    for (p0=1,l0=1; p0<MAX_URATS; ++p0,l0<<=1) {
        if (gs.tech & l0) {
            int len = _urats_len[p0];
            int pos = _urats_pos[p0];
            if (ratslen + len + 1 > sizeof(rats)) break; // for safety
            if (ratslen > 0)
                rats[ratslen++] = '+';
            memcpy(rats+ratslen, _urats + pos, len);
            ratslen += len;
        }
    }
    str = pstring_new(ratslen,rats);
    PTUPLE_SET_ITEM(tpl,0,str);

    PTUPLE_SET_ITEM(tpl,1,PSMALLINT_NEW(mcc));
    PTUPLE_SET_ITEM(tpl,2,PSMALLINT_NEW(mnc));
    str = pstring_new(0,NULL);
    PTUPLE_SET_ITEM(tpl,3,str);

    str = pstring_new(strlen(gs.lac),gs.lac);
    PTUPLE_SET_ITEM(tpl,4,str);
    str = pstring_new(strlen(gs.ci),gs.ci);
    PTUPLE_SET_ITEM(tpl,5,str);

    //registered to network
    PTUPLE_SET_ITEM(tpl,6,(gs.registered==GS_REG_OK || gs.registered==GS_REG_ROAMING) ? PBOOL_TRUE():PBOOL_FALSE());
    //attached to APN
    PTUPLE_SET_ITEM(tpl,7,gs.attached ? PBOOL_TRUE():PBOOL_FALSE());

    ACQUIRE_GIL();
    *res = tpl;
    return ERR_OK;
}

/**
 * @brief _bg96_mobile_info retrieves info on IMEI and SIM card by means of +CGSN and *CCID
 *
 *
 */
C_NATIVE(_bg96_mobile_info){
    NATIVE_UNWARN();
    uint8_t imei[16];
    uint8_t iccid[22];
    int im_len;
    int ic_len;

    PTuple *tpl = ptuple_new(2,NULL);
    RELEASE_GIL();
    im_len = _gs_imei(imei);
    ic_len = _gs_iccid(iccid);

    if(im_len<=0) {
        PTUPLE_SET_ITEM(tpl,0,pstring_new(0,NULL));
    } else {
        PTUPLE_SET_ITEM(tpl,0,pstring_new(im_len,imei));
    }
    if(ic_len<=0) {
        PTUPLE_SET_ITEM(tpl,1,pstring_new(0,NULL));
    } else {
        PTUPLE_SET_ITEM(tpl,1,pstring_new(ic_len,iccid));
    }
    ACQUIRE_GIL();
    *res =tpl;
    return ERR_OK;
}

/**
 * @brief _bg96_link_info retrieves ip and dns by means of +UPSND
 *
 *
 */
C_NATIVE(_bg96_link_info){
    NATIVE_UNWARN();
    PString *ips;
    PString *dns;
    uint32_t addrlen;
    uint8_t addrbuf[16];

    RELEASE_GIL();

    addrlen = _gs_local_ip(addrbuf);
    if(addrlen>0){
        ips = pstring_new(addrlen,addrbuf);
    } else {
        ips = pstring_new(0,NULL);
    }

    addrlen = _gs_dns(addrbuf);
    if(addrlen>0){
        dns = pstring_new(addrlen,addrbuf);
    } else {
        dns = pstring_new(0,NULL);
    }

    PTuple *tpl = ptuple_new(2,NULL);
    PTUPLE_SET_ITEM(tpl,0,ips);
    PTUPLE_SET_ITEM(tpl,1,dns);

    ACQUIRE_GIL();
    *res = tpl;
    return ERR_OK;
}



#define DRV_SOCK_DGRAM 1
#define DRV_SOCK_STREAM 0
#define DRV_AF_INET 0

C_NATIVE(_bg96_socket_create){
    NATIVE_UNWARN();
    int32_t family;
    int32_t type;
    int32_t proto;
    if (parse_py_args("III", nargs, args, DRV_AF_INET, &family, DRV_SOCK_STREAM, &type, 6 /*tcp*/, &proto) != 3) return ERR_TYPE_EXC;
    if (type != DRV_SOCK_DGRAM && type != DRV_SOCK_STREAM)
        return ERR_TYPE_EXC;
    if (family != DRV_AF_INET)
        return ERR_UNSUPPORTED_EXC;

    RELEASE_GIL();
    int32_t sock_id = gzsock_socket(
          family,
          type + 1,
          proto,
          NULL);
    ACQUIRE_GIL();
    if(sock_id<0) {
        return ERR_IOERROR_EXC;
    } 
    *res = PSMALLINT_NEW(sock_id);
    return ERR_OK;
}

C_NATIVE(_bg96_socket_connect) {
    C_NATIVE_UNWARN();
    int32_t sock,ret;
    NetAddress netaddr;
    struct sockaddr_in addr;

    if (parse_py_args("in", nargs, args, &sock, &netaddr) != 2)
        return ERR_TYPE_EXC;

    addr.sin_family = AF_INET;
    addr.sin_port = netaddr.port;
    addr.sin_addr.s_addr = netaddr.ip;

    *res = MAKE_NONE();

    RELEASE_GIL();
    ret = gzsock_connect(sock, (struct sockaddr *) &addr, sizeof(addr));
    ACQUIRE_GIL();

    if (ret < 0) {
        return ERR_CONNECTION_REF_EXC;
    }

    return ERR_OK;
}

C_NATIVE(_bg96_socket_close) {
    C_NATIVE_UNWARN();
    int32_t sock;
    int ret;
    if (parse_py_args("i", nargs, args, &sock) != 1)
        return ERR_TYPE_EXC;
    RELEASE_GIL();
    ret = gzsock_close(sock);
    ACQUIRE_GIL();
    *res = PSMALLINT_NEW(ret);
    return ERR_OK;
}

C_NATIVE(_bg96_socket_send) {
    C_NATIVE_UNWARN();
    uint8_t *buf;
    int32_t len;
    int32_t flags;
    int32_t sock;
    int ret;
    if (parse_py_args("isi", nargs, args,
                &sock,
                &buf, &len,
                &flags) != 3) return ERR_TYPE_EXC;

    RELEASE_GIL();
    ret = gzsock_send(sock, buf, len, flags);
    ACQUIRE_GIL();

    if (ret<0) return ERR_IOERROR_EXC;
    *res = PSMALLINT_NEW((uint32_t)ret);
    return ERR_OK;
}

C_NATIVE(_bg96_socket_sendto){
    C_NATIVE_UNWARN();
    uint8_t* buf;
    int32_t len;
    int32_t flags;
    int32_t sock;
    int32_t wrt=0;
    int32_t tsnd;
    int32_t ret;
    NetAddress netaddr;

    if (parse_py_args("isni", nargs, args,
            &sock,
            &buf, &len,
            &netaddr,
            &flags)
        != 4)
        return ERR_TYPE_EXC;

    struct sockaddr_in addr;

    addr.sin_family = AF_INET;
    addr.sin_port = netaddr.port;
    addr.sin_addr.s_addr = netaddr.ip;

    RELEASE_GIL();
    ret = gzsock_sendto(sock, buf, len, flags, (struct sockaddr *) &addr, sizeof(addr));
    ACQUIRE_GIL();
    if (ret<0) return ERR_IOERROR_EXC;
    *res = PSMALLINT_NEW((uint32_t)ret);
    return ERR_OK;
}

C_NATIVE(_bg96_socket_recv_into){
    C_NATIVE_UNWARN();
    uint8_t* buf;
    int32_t len;
    int32_t sz;
    int32_t flags;
    int32_t ofs;
    int32_t sock;
    int ret;

    if (parse_py_args("isiiI", nargs, args,
            &sock,
            &buf, &len,
            &sz,
            &flags,
            0,
            &ofs)
        != 5)
        return ERR_TYPE_EXC;
    buf += ofs;
    len -= ofs;
    len = (sz < len) ? sz : len;
    RELEASE_GIL();
    ret = gzsock_recv(sock, buf, len, flags);
    ACQUIRE_GIL();
    printf("requested %i from %i, returned %i\n",len,sock,ret);
    if (ret<0) {
        if (ret == ERR_TIMEOUT) {
            return ERR_TIMEOUT_EXC;
        } 
#if defined ZERYNTH_SSL || defined NATIVE_MBEDTLS
        else if (ret == MBEDTLS_ERR_SSL_TIMEOUT){
            return ERR_TIMEOUT_EXC;
        }
#endif
        else {
            return ERR_IOERROR_EXC;
        }
    }
    
    *res = PSMALLINT_NEW((uint32_t)ret);
    return ERR_OK;
}

C_NATIVE(_bg96_socket_recvfrom_into){
    C_NATIVE_UNWARN();
    uint8_t* buf;
    int32_t len;
    int32_t sz;
    int32_t flags;
    int32_t ofs;
    int32_t sock;
    int ret;

    PString *oaddr = NULL;
    int32_t port;

    struct sockaddr_in addr;

    if (parse_py_args("isiiI", nargs, args,
            &sock,
            &buf, &len,
            &sz,
            &flags,
            0,
            &ofs)
        != 5)
        return ERR_TYPE_EXC;
    buf += ofs;
    len -= ofs;
    len = (sz < len) ? sz : len;
    RELEASE_GIL();
    ret = gzsock_recvfrom(sock, buf, len, flags,  (struct sockaddr *) &addr, sizeof(addr));
    ACQUIRE_GIL();
    if(ret>=0){
        uint8_t remote_ip[16];
        int saddrlen;
        saddrlen = zs_addr_to_string(&addr,remote_ip);
        PTuple* tpl = (PTuple*)psequence_new(PTUPLE, 2);
        PTUPLE_SET_ITEM(tpl, 0, PSMALLINT_NEW(ret));
        oaddr = pstring_new(saddrlen,remote_ip);
        PTuple* ipo = ptuple_new(2,NULL);
        PTUPLE_SET_ITEM(ipo,0,oaddr);
        PTUPLE_SET_ITEM(ipo,1,PSMALLINT_NEW(OAL_GET_NETPORT(addr.sin_port)));
        PTUPLE_SET_ITEM(tpl, 1, ipo);
        *res = tpl;
    }
    return ERR_OK;
}



C_NATIVE(_bg96_socket_bind){
    C_NATIVE_UNWARN();
    int32_t sock;
    NetAddress netaddr;
    struct sockaddr_in addr;
    int ret;
    if (parse_py_args("in", nargs, args, &sock, &netaddr) != 2)
        return ERR_TYPE_EXC;

    addr.sin_family = AF_INET;
    addr.sin_port = netaddr.port;
    addr.sin_addr.s_addr = netaddr.ip;
    RELEASE_GIL();
    ret = gzsock_bind(sock,&addr,sizeof(addr));
    ACQUIRE_GIL();
    if(ret<0) {
        return ERR_IOERROR_EXC;
    }
    *res = MAKE_NONE();
    return ERR_OK;
}


C_NATIVE(_bg96_socket_select){
    C_NATIVE_UNWARN();
    int32_t timeout;
    int32_t tmp, i, j, sock = -1,ret;

    if (nargs < 4)
    return ERR_TYPE_EXC;

    fd_set rfd;
    fd_set wfd;
    fd_set xfd;
    struct timeval tms;
    struct timeval *ptm;
    PObject *rlist = args[0];
    PObject *wlist = args[1];
    PObject *xlist = args[2];
    fd_set *fdsets[3] = {&rfd, &wfd, &xfd};
    PObject *slist[3] = {rlist, wlist, xlist};
    PObject *tm = args[3];


    if (tm == MAKE_NONE()) {
        ptm = NULL;
    } else if (IS_PSMALLINT(tm)) {
        timeout = PSMALLINT_VALUE(tm);
        if (timeout < 0)
            return ERR_TYPE_EXC;
        tms.tv_sec = timeout / 1000;
        tms.tv_usec = (timeout % 1000) * 1000;
        ptm = &tms;
    } else return ERR_TYPE_EXC;

    for (j = 0; j < 3; j++) {
        tmp = PTYPE(slist[j]);
        if (!IS_OBJ_PSEQUENCE_TYPE(tmp))
            return ERR_TYPE_EXC;
        FD_ZERO (fdsets[j]);
        for (i = 0; i < PSEQUENCE_ELEMENTS(slist[j]); i++) {
            PObject *fd = PSEQUENCE_OBJECTS(slist[j])[i];
            if (IS_PSMALLINT(fd)) {
                //printf("%i -> %i\n",j,PSMALLINT_VALUE(fd));
                FD_SET(PSMALLINT_VALUE(fd), fdsets[j]);
                if (PSMALLINT_VALUE(fd) > sock)
                    sock = PSMALLINT_VALUE(fd);
            } else return ERR_TYPE_EXC;
        }
    }

    printf("maxsock %i\n", sock);

    RELEASE_GIL();
    ret = gzsock_select( (sock + 1), fdsets[0], fdsets[1], fdsets[2], ptm );
    ACQUIRE_GIL();

    printf("result: %i\n", ret);

    if (ret < 0) {
        return ERR_IOERROR_EXC;
    }

    PTuple *tpl = (PTuple *) psequence_new(PTUPLE, 3);
    for (j = 0; j < 3; j++) {
        tmp = 0;
        for (i = 0; i <= sock; i++) {
            if (FD_ISSET(i, fdsets[j])) tmp++;
        }
        PTuple *rtpl = psequence_new(PTUPLE, tmp);
        tmp = 0;
        for (i = 0; i <= sock; i++) {
            //printf("sock %i in %i = %i\n",i,j,FD_ISSET(i, fdsets[j]));
            if (FD_ISSET(i, fdsets[j])) {
                PTUPLE_SET_ITEM(rtpl, tmp, PSMALLINT_NEW(i));
                tmp++;
            }
        }
        PTUPLE_SET_ITEM(tpl, j, rtpl);
    }
    *res = tpl;
    return ERR_OK;

}

// /////////////////////SSL/TLS

#define _CERT_NONE 1
#define _CERT_OPTIONAL 2
#define _CERT_REQUIRED 4
#define _CLIENT_AUTH 8
#define _SERVER_AUTH 16

C_NATIVE(_bg96_secure_socket) {
    C_NATIVE_UNWARN();
#if defined ZERYNTH_SSL || defined NATIVE_MBEDTLS
    int32_t err = ERR_OK;
    int32_t sock;
    int32_t i;
    SSLInfo nfo;

    int32_t family = DRV_AF_INET;
    int32_t type = DRV_SOCK_STREAM;
    int32_t proto = 6;
    int32_t ssocknum = 0;
    int32_t ctxlen;
    uint8_t* certbuf = NULL;
    uint16_t certlen = 0;
    uint8_t* clibuf = NULL;
    uint16_t clilen = 0;
    uint8_t* pkeybuf = NULL;
    uint16_t pkeylen = 0;
    uint32_t options = _CLIENT_AUTH | _CERT_NONE;
    uint8_t* hostbuf = NULL;
    uint16_t hostlen = 0;

    PTuple* ctx;
    memset(&nfo,0,sizeof(nfo));
    ctx = (PTuple*)args[nargs - 1];
    nargs--;
    
    if (parse_py_args("III", nargs, args, DRV_AF_INET, &family, DRV_SOCK_STREAM, &type, 6, &proto) != 3){
        return ERR_TYPE_EXC;
    }
    if (type != DRV_SOCK_DGRAM && type != DRV_SOCK_STREAM){
        return ERR_TYPE_EXC;
    }
    if (family != DRV_AF_INET)
        return ERR_UNSUPPORTED_EXC;
    if(proto!=6)
        return ERR_UNSUPPORTED_EXC;

    ctxlen = PSEQUENCE_ELEMENTS(ctx);
    if (ctxlen && ctxlen != 5)
        return ERR_TYPE_EXC;

    if (ctxlen) {
        //ssl context passed
        PObject* cacert = PTUPLE_ITEM(ctx, 0);
        PObject* clicert = PTUPLE_ITEM(ctx, 1);
        PObject* ppkey = PTUPLE_ITEM(ctx, 2);
        PObject* host = PTUPLE_ITEM(ctx, 3);
        PObject* iopts = PTUPLE_ITEM(ctx, 4);

        nfo.cacert = PSEQUENCE_BYTES(cacert);
        nfo.cacert_len = PSEQUENCE_ELEMENTS(cacert);
        nfo.clicert = PSEQUENCE_BYTES(clicert);
        nfo.clicert_len = PSEQUENCE_ELEMENTS(clicert);
        nfo.hostname = PSEQUENCE_BYTES(host);
        nfo.hostname_len = PSEQUENCE_ELEMENTS(host);
        nfo.pvkey = PSEQUENCE_BYTES(ppkey);
        nfo.pvkey_len = PSEQUENCE_ELEMENTS(ppkey);
        nfo.options = PSMALLINT_VALUE(iopts);
    }
    RELEASE_GIL();
    sock = gzsock_socket(
          family,
          type,
          proto,
          (ctxlen) ? &nfo:NULL);
    ACQUIRE_GIL();
    printf("CMD_SOCKET: %i %i\n", sock, type);
    if (sock < 0)
      return ERR_IOERROR_EXC;
    *res = PSMALLINT_NEW(sock);
    return ERR_OK;    

#else 
    int32_t err = ERR_OK;
    int32_t family = DRV_AF_INET;
    int32_t type = DRV_SOCK_STREAM;
    int32_t proto = 6;
    int32_t sock;
    int32_t i;
    int32_t ssocknum = 0;
    int32_t ctxlen;
    uint8_t* certbuf = NULL;
    uint16_t certlen = 0;
    uint8_t* clibuf = NULL;
    uint16_t clilen = 0;
    uint8_t* pkeybuf = NULL;
    uint16_t pkeylen = 0;
    uint32_t options = _CLIENT_AUTH | _CERT_NONE;
    uint8_t* hostbuf = NULL;
    uint16_t hostlen = 0;
    int amode;

    PTuple* ctx;
    ctx = (PTuple*)args[nargs - 1];
    nargs--;
    if (parse_py_args("III", nargs, args, DRV_AF_INET, &family, DRV_SOCK_STREAM, &type, 6, &proto) != 3){
        return ERR_TYPE_EXC;
    }
    if (type != DRV_SOCK_DGRAM && type != DRV_SOCK_STREAM){
        return ERR_TYPE_EXC;
    }
    if (family != DRV_AF_INET)
        return ERR_UNSUPPORTED_EXC;
    if(proto!=6)
        return ERR_UNSUPPORTED_EXC;

    ctxlen = PSEQUENCE_ELEMENTS(ctx);
    if (ctxlen && ctxlen != 5)
        return ERR_TYPE_EXC;

    if (ctxlen) {
        //ssl context passed
        PObject* cacert = PTUPLE_ITEM(ctx, 0);
        PObject* clicert = PTUPLE_ITEM(ctx, 1);
        PObject* ppkey = PTUPLE_ITEM(ctx, 2);
        PObject* host = PTUPLE_ITEM(ctx, 3);
        PObject* iopts = PTUPLE_ITEM(ctx, 4);
        certbuf = PSEQUENCE_BYTES(cacert);
        certlen = PSEQUENCE_ELEMENTS(cacert);
        clibuf = PSEQUENCE_BYTES(clicert);
        clilen = PSEQUENCE_ELEMENTS(clicert);
        hostbuf = PSEQUENCE_BYTES(host);
        hostlen = PSEQUENCE_ELEMENTS(host);
        pkeybuf = PSEQUENCE_BYTES(ppkey);
        pkeylen = PSEQUENCE_ELEMENTS(ppkey);
        options = PSMALLINT_VALUE(iopts);
    }

    GSocket* sslsock = NULL;


    RELEASE_GIL();
    sock = _gs_socket_new(proto,1);  //request secure socket
    if(sock<0) {
        err = ERR_IOERROR_EXC;
        *res = MAKE_NONE();
    } else {
        *res = PSMALLINT_NEW(sock);
        err = ERR_OK;
    }
    if(err==ERR_OK) {
        //let's configure tls

        //NOTE: ZERYNTH CERTS END with \0
        if(options&_CERT_NONE) {
           //NO CACERT VERIFICATION
           amode = 0;
        } else {
            //REQUIRED OR OPTIONAL AUTH
            if(certlen) {
                //CACERT GIVEN
                amode=1;
                certlen--;
            }else {
                //NO CACERT
                amode=1;
            }
        }
        if(clilen) {
            //load clicert
            amode=2;
            clilen--;
        }
        if(pkeylen){
            //load clipkey
            amode=2;
            pkeylen--;
        }

        if (_gs_socket_tls(sock,certbuf,certlen,clibuf,clilen,pkeybuf,pkeylen,amode)) {
            //oops, close socket
            _gs_socket_closing(sock);
            err = ERR_IOERROR_EXC;
        }
    }

    ACQUIRE_GIL();
    return err;
#endif
}



// /////////////////////DNS

C_NATIVE(_bg96_resolve){
    C_NATIVE_UNWARN();
    uint8_t* url;
    uint32_t len;
    uint8_t saddr[16];
    uint32_t saddrlen;
    int ret;
    if (parse_py_args("s", nargs, args, &url, &len) != 1)
        return ERR_TYPE_EXC;

    // if arg is already numeric IP address, return it!
    struct sockaddr_in addr;
    ret = zs_string_to_addr(url,len,&addr);
    if (ret == ERR_OK) {
        *res = args[0];
        return ERR_OK;
    }
    // otherwise resolve IP address
    struct addrinfo *ip;
    uint8_t *node = gc_malloc(len+1);
    memcpy(node,url,len);
    node[len] = 0; //get a zero terminated string
    RELEASE_GIL();
    ret = zsock_getaddrinfo(node,NULL,NULL,&ip);
    gc_free(node);
    ACQUIRE_GIL();
    if (ret==ERR_OK) {
        saddrlen=zs_addr_to_string(ip->ai_addr,saddr);
        zsock_freeaddrinfo(ip);
        if(saddrlen>0) {
            *res = pstring_new(saddrlen,saddr);
            return ERR_OK;
        } 
    }
    return ERR_IOERROR_EXC;
}



// /////////////////////RTC

C_NATIVE(_bg96_rtc){
    C_NATIVE_UNWARN();
    int err = ERR_OK;
    uint8_t time[20];
    *res = MAKE_NONE();
    memset(time,0,20);
    RELEASE_GIL();
    if(!_gs_get_rtc(time)) err=ERR_RUNTIME_EXC;
    ACQUIRE_GIL();
    if (err==ERR_OK) {
        PTuple* tpl = ptuple_new(7,NULL);
        int yy,MM,dd,hh,mm,ss,tz;
        yy = 2000+((time[0]-'0')*10+(time[1]-'0'));
        MM = (time[3]-'0')*10+(time[4]-'0');
        dd = (time[6]-'0')*10+(time[7]-'0');
        hh = (time[9]-'0')*10+(time[10]-'0');
        mm = (time[12]-'0')*10+(time[13]-'0');
        ss = (time[15]-'0')*10+(time[16]-'0');
        tz = ((time[18]-'0')*10+(time[19]-'0'))*15*((time[17]=='-')?-1:1);
        PTUPLE_SET_ITEM(tpl,0,PSMALLINT_NEW(yy));
        PTUPLE_SET_ITEM(tpl,1,PSMALLINT_NEW(MM));
        PTUPLE_SET_ITEM(tpl,2,PSMALLINT_NEW(dd));
        PTUPLE_SET_ITEM(tpl,3,PSMALLINT_NEW(hh));
        PTUPLE_SET_ITEM(tpl,4,PSMALLINT_NEW(mm));
        PTUPLE_SET_ITEM(tpl,5,PSMALLINT_NEW(ss));
        PTUPLE_SET_ITEM(tpl,6,PSMALLINT_NEW(tz));
        *res = tpl;
    }
    return err;
}


///////////////////////SMS
C_NATIVE(_bg96_sms_send){
    NATIVE_UNWARN();
    int32_t err = ERR_OK;
    int32_t numlen;
    int32_t txtlen;
    int32_t mr;
    uint8_t* num;
    uint8_t* txt;
    *res = MAKE_NONE();
    
    if(parse_py_args("ss",nargs,args,&num,&numlen,&txt,&txtlen)!=2) return ERR_TYPE_EXC;

    RELEASE_GIL();
    mr = _gs_sms_send(num,numlen,txt,txtlen);
    ACQUIRE_GIL();

    if (mr == -1)
        *res = PSMALLINT_NEW(-1);
    else if (mr < 0)
        err = bg96exc;
    else
        *res = pinteger_new(mr);
    return err;
}

C_NATIVE(_bg96_sms_list){
    NATIVE_UNWARN();
    int32_t err = ERR_OK;
    int32_t unread;
    int32_t maxsms;
    int32_t offset;
    int32_t msgcnt;
    int i;
    *res = MAKE_NONE();
    
    if(parse_py_args("iii",nargs,args,&unread,&maxsms,&offset)!=3) return ERR_TYPE_EXC;

    GSSMS *sms = gc_malloc(sizeof(GSSMS)*maxsms);
    RELEASE_GIL();
    msgcnt = _gs_sms_list(unread,sms,maxsms,offset);
    ACQUIRE_GIL();

    PTuple *tpl = ptuple_new(msgcnt,NULL);
    for(i=0;i<msgcnt;i++){
        GSSMS *sm = &sms[i];
        PTuple *pres = ptuple_new(4,NULL);
        PTUPLE_SET_ITEM(pres,0,pstring_new(sm->txtlen,sm->txt));
        PTUPLE_SET_ITEM(pres,1,pstring_new(sm->oaddrlen,sm->oaddr));
        if (sm->tslen<22) {
            //bad ts
            PTUPLE_SET_ITEM(pres,2,ptuple_new(0,NULL));
        } else {
            PTuple *tm = ptuple_new(7,NULL);
            int nn=0;

            nn = (sm->ts[0]-'0')*1000+(sm->ts[1]-'0')*100+(sm->ts[2]-'0')*10+(sm->ts[3]-'0');
            PTUPLE_SET_ITEM(tm,0,PSMALLINT_NEW(nn));
            nn = (sm->ts[5]-'0')*10+(sm->ts[6]-'0');
            PTUPLE_SET_ITEM(tm,1,PSMALLINT_NEW(nn));
            nn = (sm->ts[8]-'0')*10+(sm->ts[9]-'0');
            PTUPLE_SET_ITEM(tm,2,PSMALLINT_NEW(nn));
            nn = (sm->ts[11]-'0')*10+(sm->ts[12]-'0');
            PTUPLE_SET_ITEM(tm,3,PSMALLINT_NEW(nn));
            nn = (sm->ts[14]-'0')*10+(sm->ts[15]-'0');
            PTUPLE_SET_ITEM(tm,4,PSMALLINT_NEW(nn));
            nn = (sm->ts[17]-'0')*10+(sm->ts[18]-'0');
            PTUPLE_SET_ITEM(tm,5,PSMALLINT_NEW(nn));
            nn = (sm->ts[20]-'0')*10+(sm->ts[21]-'0');
            PTUPLE_SET_ITEM(tm,6,PSMALLINT_NEW(nn*15));
            PTUPLE_SET_ITEM(pres,2,tm);
        }
    
        PTUPLE_SET_ITEM(pres,3,PSMALLINT_NEW(sm->index));

        PTUPLE_SET_ITEM(tpl,i,pres);
    }
    *res= tpl;
    gc_free(sms);
    return err;
}

C_NATIVE(_bg96_sms_pending){
    NATIVE_UNWARN();
    *res = PSMALLINT_NEW(gs.pendingsms);
    return ERR_OK;
}

C_NATIVE(_bg96_sms_delete){
    NATIVE_UNWARN();
    int32_t err = ERR_OK;
    int32_t index, rd;
    *res = PBOOL_TRUE();

    if(parse_py_args("i",nargs,args,&index)!=1) return ERR_TYPE_EXC;

    RELEASE_GIL();
    rd = _gs_sms_delete(index);
    ACQUIRE_GIL();

    if (rd<0) *res=PBOOL_FALSE();
    return err;
}

C_NATIVE(_bg96_sms_get_scsa){
    NATIVE_UNWARN();
    int32_t err = ERR_OK;
    int32_t scsalen;
    uint8_t scsa[32];


    RELEASE_GIL();
    scsalen = _gs_sms_get_scsa(scsa);
    if (scsalen<0) scsalen = 0;
    *res = pstring_new(scsalen,scsa);
    ACQUIRE_GIL();
    return err;
}

C_NATIVE(_bg96_sms_set_scsa){
    NATIVE_UNWARN();
    int32_t err = ERR_OK;
    int32_t scsalen,rd;
    uint8_t *scsa;
    *res = PBOOL_TRUE();

    if(parse_py_args("s",nargs,args,&scsa,&scsalen)!=1) return ERR_TYPE_EXC;

    RELEASE_GIL();
    rd = _gs_sms_set_scsa(scsa,scsalen);
    ACQUIRE_GIL();

    if (rd<0) *res = PBOOL_FALSE();
    return err;
}


///////////////////////GNSS

C_NATIVE(_bg96_gnss_init){
    C_NATIVE_UNWARN();
    int err = ERR_OK;
    int use_uart3 = 0;
    int gnss_rate;
    *res = MAKE_NONE();

    if(parse_py_args("ii",nargs,args,&gnss_rate,&use_uart3)!=2) return ERR_TYPE_EXC;

    RELEASE_GIL();

    if(!gs.running) err=ERR_PERIPHERAL_INVALID_HARDWARE_STATUS_EXC;
    else
    if(_gs_gnss_init(gnss_rate, use_uart3)!=0) err=ERR_RUNTIME_EXC;

    ACQUIRE_GIL();
    return err;
}

C_NATIVE(_bg96_gnss_done){
    C_NATIVE_UNWARN();
    int err = ERR_OK;
    *res = MAKE_NONE();

    RELEASE_GIL();

    if(!gs.running) err=ERR_PERIPHERAL_INVALID_HARDWARE_STATUS_EXC;
    else
    if(_gs_gnss_done()!=0) err=ERR_RUNTIME_EXC;

    ACQUIRE_GIL();
    return err;
}

C_NATIVE(_bg96_gnss_fix){
    C_NATIVE_UNWARN();
    int err = ERR_OK;
    int r;
    GNSSLoc loc;
    *res = MAKE_NONE();

    RELEASE_GIL();
    r = _gs_gnss_loc(&loc);
    if(r==0) {
        //there is a fix, let's decode
        PTuple *tpl = ptuple_new(10,NULL);
        PTuple *utc = ptuple_new(6,NULL);

        PTUPLE_SET_ITEM(utc,0,PSMALLINT_NEW((2000+loc.yy)));
        PTUPLE_SET_ITEM(utc,1,PSMALLINT_NEW(loc.MM));
        PTUPLE_SET_ITEM(utc,2,PSMALLINT_NEW(loc.dd));
        PTUPLE_SET_ITEM(utc,3,PSMALLINT_NEW(loc.hh));
        PTUPLE_SET_ITEM(utc,4,PSMALLINT_NEW(loc.mm));
        PTUPLE_SET_ITEM(utc,5,PSMALLINT_NEW(loc.ss));

        PTUPLE_SET_ITEM(tpl,0,pfloat_new(loc.lat));
        PTUPLE_SET_ITEM(tpl,1,pfloat_new(loc.lon));
        PTUPLE_SET_ITEM(tpl,2,pfloat_new(loc.alt));
        PTUPLE_SET_ITEM(tpl,3,pfloat_new(loc.speed));
        PTUPLE_SET_ITEM(tpl,4,pfloat_new(loc.cog));
        PTUPLE_SET_ITEM(tpl,5,PSMALLINT_NEW(loc.nsat));
        PTUPLE_SET_ITEM(tpl,6,pfloat_new(loc.precision));
        PTUPLE_SET_ITEM(tpl,7,pfloat_new(-1.0));
        PTUPLE_SET_ITEM(tpl,8,pfloat_new(-1.0));
        PTUPLE_SET_ITEM(tpl,9,utc);
        *res = tpl;
    }
    ACQUIRE_GIL();
    return err;
}
