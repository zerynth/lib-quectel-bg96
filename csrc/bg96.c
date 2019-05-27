/**
 * @file bg96.c
 * @brief Driver for Quectel BG96 modules
 * @author Giacomo Baldi
 * @version 
 * @date 2019-03-07
 */



/** \mainpage Driver Architecture
 * 
 * This driver consists of:
 *
 * - a main thread (_gs_loop) that has exclusive access to the serial port in input
 * - a mechanism based on slots (GSlot) such that each thread calling into the driver must wait its turn to issue an AT command
 * - a list of socket structures (GSocket)
 *
 * The main thread reads one line at time and checks if it is a command response or not. In case it is a command response, it tries
 * to handle it based on the current slot. If the command response is a URC, it is handled by the corresponding functon (_gs_handle_urc),
 * otherwise if the line is not a command response, it is checked against "OK", "+CME ERROR", "ERROR" or ">" and action on the current slot
 * is taken.
 *
 * Once a slot is acquired for a particular command, the following can happen:
 *
 * - an OK is received and the thread owning the slot is signaled
 * - an error condition is received and the thread owning the slot is signaled
 * - the slot timeout is reached and the thread owning the slot is signaled
 * - a valid command response is detected and the gs.buffer is copied to the slot buffer for parsing
 * 
 * In all cases it is not possible for a slot to stall the main thread longer than timeout milliseconds. Obviously the thread owning
 * the slot must behave correctly:
 *
 * - acquire the slot
 * - send AT command
 * - wait for main thread signal
 * - parse the arguments from command if present/needed
 * - release the slot
 *
 * After slot release, the slot memory is no longer valid, unless allocated by the calling thread.
 *
 *
 */



#include "zerynth.h"
#include "bg96.h"


//Enable/Disable debug printf
#define QUECTEL_BG96_DEBUG 0

#if QUECTEL_BG96_DEBUG
#define printf(...) vbl_printf_stdout(__VA_ARGS__)
#else
#define printf(...)
#endif



//STATIC VARIABLES

//the BG96 driver status
GStatus gs;
//the list of available sockets
static GSocket gs_sockets[MAX_SOCKS];
//the one and only slot available to threads
//to get the bg96 driver attention
static GSSlot gslot;
//the list of GSM operators
GSOp gsops[MAX_OPS];
//the number of GSM operators
int gsopn=0;


//Some declarations for URC socket handling
void _gs_socket_closing(int id);
void _gs_socket_pending(int id);


/**
 * @brief Initializes the data structures of bg96
 */
void _gs_init(){
    int i;
    printf("Initializing GSM\n");
    for(i=0;i<MAX_SOCKS;i++){
        gs_sockets[i].lock = vosSemCreate(1);
        gs_sockets[i].rx = vosSemCreate(0);
    }
    memset(&gs,0,sizeof(GStatus));
    gs.slotlock = vosSemCreate(1);
    gs.sendlock = vosSemCreate(1);
    gs.slotdone = vosSemCreate(0);
    gs.bufmode = vosSemCreate(0);
    gs.dnsmode = vosSemCreate(1);
    gs.secure_sock_id = -1;
    gs.initialized=0;
}

/**
 * @brief Clean up the data structures of bg96
 */
void _gs_done(){
    int i;
    vhalSerialDone(gs.serial);
}


/**
 * @brief Begin the power off phase
 *
 * Starts power off sequence according to https://www.quectel.com/UploadImage/Downlad/Quectel_BG96_Hardware_Design_V1.3.pdf
 *
 *
 * @return 0 on success
 */
int _gs_poweroff(){
    int i;
    
    printf("Powering off...\n");
    if(!vhalPinRead(gs.status)) {
        //status at 0, already powered down
        printf("STA 0\n");
        vosThSleep(TIME_U(3000,MILLIS));
        return 0;
    }

    printf("PWR 1\n");
    vhalPinWrite(gs.poweron,1);
    vosThSleep(TIME_U(800,MILLIS));
    printf("PWR 0\n");
    vhalPinWrite(gs.poweron,0);
    vosThSleep(TIME_U(800,MILLIS)); // >= 650ms
    // vhalPinWrite(gs.poweron,1);
    printf("PWR 1\n");
    //wait for power down to complete checking the status pin
    for (i=0;i<50;i++){
        if(!vhalPinRead(gs.status)) {
            printf("STA 0\n");
            //status at 0, exit after waiting 
            break;
        }
        printf("STA 1\n");
        vosThSleep(TIME_U(100,MILLIS));
    }
    vosThSleep(TIME_U(1000,MILLIS));
    //exit after 3seconds if status stays high for safety
    return 0;
}

int _gs_wait_for_ready(int timeout){
    while(_gs_readline(timeout)>=0){
        if(_gs_check_rdy()){
            return 1;
        }
    }
    return 0;
}

/**
 * @brief Begin the power up phase
 *
 * Starts power on sequence according to https://www.quectel.com/UploadImage/Downlad/Quectel_BG96_Hardware_Design_V1.3.pdf
 *
 * @return 0 on success
 */
int _gs_poweron(){
    int i;


    _gs_poweroff();
        
    printf("Powering on...%x\n",gs.poweron);
    printf("PWR 1\n");
    vhalPinWrite(gs.poweron,1);
    vosThSleep(TIME_U(800,MILLIS));  //wait for stabilization
    printf("PWR 0\n");
    vhalPinWrite(gs.poweron,0);
    vosThSleep(TIME_U(800,MILLIS));  // >= 500ms
    printf("PWR 1\n");
    vhalPinWrite(gs.poweron,1);
    for(i=0;i<50;i++){
        if(vhalPinRead(gs.status)) {
            //status at 1, exit
            printf("STA 1\n");
            break;
        }
        printf("STA 0 %x\n",gs.status);
        vosThSleep(TIME_U(100,MILLIS));
    }
    if(!vhalPinRead(gs.status)) {
        //status at 0, can't power up
        printf("power on ko\n");
        // return 1;
    }



    // vhalPinWrite(gs.dtr,0);
    // vhalPinWrite(gs.rts,0);
    return 1;
}

/**
 * @brief Read lines from the module until a "OK" is received
 *
 * @param[in]   timeout     the number of milliseconds to wait for each line
 *
 * @return 0 on failure
 */
int _gs_wait_for_ok(int timeout){
    while(_gs_readline(timeout)>=0){
        if(_gs_check_ok()){
            return 1;
        }
    }
    return 0;
}

/**
 * @brief Read a line from the module
 *
 * Lines are saved into gs.buffer and null terminated. The number of bytes read 
 * is saved in gs.buffer and returned. The timeout is implemented with a 50 milliseconds
 * polling strategy. TODO: change when the serial driver will support timeouts
 *
 * @param[in]   timeout     the number of milliseconds to wait for a line
 *
 * @return the number of bytes read or -1 on timeout
 */
int _gs_readline(int timeout){
    gs.bytes = 0;
    memset(gs.buffer,0,16);
    uint8_t *buf = gs.buffer;
    uint32_t tstart = vosMillis();
    while(gs.bytes<(MAX_BUF-1)){
        if(timeout>0) {
            if ((vosMillis()-tstart)>timeout) {
                *buf=0;
                return -1;
            }
            if(vhalSerialAvailable(gs.serial)>0){
                vhalSerialRead(gs.serial,buf,1);
            } else {
                vosThSleep(TIME_U(50,MILLIS));
                continue;
            }
        } else {
            vhalSerialRead(gs.serial,buf,1);
        }
        gs.bytes++;
//        printf("->%i\n",gs.bytes);
        if (*buf++=='\n') break;
    }
    //terminate for debugging!
    *buf=0;
    printf("rl: %s",gs.buffer);
    return gs.bytes;
}

/**
 * @brief 
 *
 * @param[in]   bytes   the number of bytes to read. If non positive, read the available ones.
 *
 *  Bytes are saved in gs.buffer and gs.bytes updated accordingly.
 *
 * @return the number of bytes read
 */
int _gs_read(int bytes){
    memset(gs.buffer,0,16);
    if (bytes<=0) bytes = vhalSerialAvailable(gs.serial);
    vhalSerialRead(gs.serial,gs.buffer,bytes);
    gs.bytes = bytes;
    gs.buffer[gs.bytes+1]=0;
    printf("rn: %s||\n",gs.buffer);
    return gs.bytes;
}

/**
 * @brief Checks if gs.buffer contains a valid "OK\r\n"
 *
 * @return 0 on failure
 */
int _gs_check_ok(){
    return memcmp(gs.buffer,"OK\r\n",4)==0 && gs.bytes>=4;
}
int _gs_check_rdy(){
    return memcmp(gs.buffer,"RDY\r\n",5)==0 && gs.bytes>=5;
}

/**
 * @brief Checks if gs.buffer contains a valid error message
 *
 * Valid error messages may come from "+CME ERROR: " responses or
 * from "ERROR" responses (+USOxxx commands). Messages from "+CME" are
 * saved in gs.errmsg up to MAX_ERR_LEN.
 *
 * @return 0 on no error
 */
int _gs_check_error(){
    if (memcmp(gs.buffer,"+CME ERROR: ",12)==0 && gs.bytes>=12){
        int elen = MIN(gs.bytes-12,MAX_ERR_LEN);
        memcpy(gs.errmsg,gs.buffer+12,elen);
        gs.errlen = elen;
        return 1;
    } else if  (memcmp(gs.buffer,"ERROR",5)==0 && gs.bytes>=5) {
        gs.errlen=0;
        return 1;
    }
    return 0;
}


/**
 * @brief Checks if gs.buffer contains a known command response
 *
 * A binary search is performed on the known command, trying to match them
 * to gs.buffer.
 *
 * @return NULL on failure, a pointer to a GSCmd structure otherwise
 */
GSCmd* _gs_parse_command_response(){
    int e0=0,e1=KNOWN_COMMANDS-1,c=-1,r=0;
    GSCmd *cmd=NULL;
    
    printf("CMD %c%c%c%c%c%c%c%c\n",gs.buffer[0],gs.buffer[1],gs.buffer[2],gs.buffer[3],gs.buffer[4],gs.buffer[5],gs.buffer[6],gs.buffer[7]);
    while(e0<=e1){
        c=(e0+e1)/2;
        cmd = &gs_commands[c];
        //for this to work the first 16 bytes of gs.buffer must be zeroed at each read!
        //otherwise previous bytes can interfere
        r= memcmp(gs.buffer,cmd->body,cmd->len);
        if (r==0 && gs.buffer[cmd->len]!=':') {
            //ouch! it means cmd is a prefix of a longer command -_-
            //so command is less than gs.buffer
            //therefore set r to +1
            printf("OUCH!\n");
            r=1;
        }
        if (r>0) e0=c+1;
        else if (r<0) e1=c-1;
        else break;
    }
    if(e0<=e1) {
        printf("RET CMD %i\n",c);
        return cmd;
    }
    printf("NULL cmd\n");
    return NULL;
}

/**
 * @brief scans buf for bytes contained in pattern
 *
 * @param[in] buf       where to start the scan
 * @param[in] ebuf      where to end the scan
 * @param[in] pattern   characters to stop to
 *
 * @return a pointer to the location of one of the bytes in pattern or NULL if they cannot be found
 */
uint8_t* _gs_advance_to(uint8_t *buf,uint8_t *ebuf,uint8_t *pattern){
    uint8_t *pt;
    while(buf<ebuf){
        pt = pattern;
        while(*pt) {
            if(*buf==*pt) return buf;
            pt++;
        }
        buf++;
    }
    return NULL;
}

/**
 * @brief parse numbers in base 10 from a byte buffe 
 *
 * Does not check for number format correctness (0003 is ok) and
 * does not parse negative numbers. Skip spaces and \r \n without checking
 * if they break a number or not ("33 44" is parsed as 3344)
 *
 * @param[in]  buf     starting point
 * @param[in]  ebuf    ending point (not included)
 * @param[out] result  the positon reached after the parsing, NULL on failed parsing
 *
 * @return 
 */
uint8_t* _gs_parse_number(uint8_t *buf, uint8_t *ebuf, int32_t *result){
    int res = 0;
    while(buf<ebuf){
        if(*buf>='0' && *buf<='9'){
            res = res*10+(*buf-'0');
        } else if(*buf!=' ' && *buf!='\r' && *buf!='\n') return NULL; //allow spaces
        buf++;
    }
    if(result) *result = res;
    return buf;
}


/**
 * @brief parse the arguments of a command response
 *
 * Parse from buf to ebuf according to the fmt string. The fmt can contain
 * only "i" and "s". "i" needs a pointer to an int as the corresponsing variadic argument
 * while "s" needs two arguments: a uint8_t** to store the pointer to the string and an int32_t* to store
 * the string length. Parameters in buf are delimited by delimiters in this pattern: ",\r\n". Strings are not copied,
 * buf is modified in place by null terminating each parameter at the rightmost delimiter, and a pointer to the string parameter
 * is returned.
 *
 * Since buf is modified, arguments can be parsed only once. TODO: consider removing null termination since it is a feature
 * needed for debug only (printf)
 *
 * @param[in]  buf   starting point
 * @param[in]  ebuf  ending point (not included)
 * @param[in]  fmt   format string
 * @param[out] ...   variadic arguments (if NULL are parsed but not returned)
 *
 * @return the number of parameters parsed
 */
int _gs_parse_command_arguments(uint8_t *buf, uint8_t *ebuf, const char*fmt,...){
    va_list vl;
    va_start(vl, fmt);
    int32_t ret=0;
    int32_t *iparam;
    uint8_t **sparam;
    uint8_t *pms;
    uint8_t *pme=ebuf;
    int i;
    pms = buf;
    while(buf<ebuf){
        buf = _gs_advance_to(buf,ebuf,",\r\n");
        if(!buf) break;
        pme = buf-1;
        // *buf=0;
        switch(*fmt) {
            case 0:
                goto exit;
            case 'i':
                iparam = va_arg(vl,int32_t*);
                if(iparam) *iparam=0;
                pms = _gs_parse_number(pms,pme+1,iparam);
                if(!pms) goto exit;
                ret++;
            break;
            case 's':
                sparam = va_arg(vl,uint8_t**);
                iparam = va_arg(vl,int32_t*);
                if(sparam) *sparam = pms;
                // *buf=0; //end string
                if(iparam) *iparam = (pme-pms)+1; //store len
                ret++;
                break;
        }
        fmt++;
        pms=++buf;
    }

exit:
    va_end(vl);
    return ret;
}



/**
 * @brief send AT command to the module
 *
 * Send to serial an AT command identified by cmd_id (see g350.h).
 * Every byte following the command can be passed in fmt. If a byte in fmt equals "i"
 * an integer is expected as a variadic argument and "i" is expanded to the string representation
 * in base 10 of such integer. If a byte in fmt equals "s", two variadic arguments are expected: a uint8_t*
 * pointing to a byte buffer and an integer containing the number of bytes to copy in place of "s". Each other byte in fmt
 * is sent as is.
 *
 * @param[i] cmd_id  Macro identifying the command to send
 * @param[i] fmt     format string
 * @param[i] ...     variadic arguments
 */
void _gs_send_at(int cmd_id,const char *fmt,...){
    static uint8_t _strbuf[16];
    GSCmd *cmd = GS_GET_CMD(cmd_id);
    uint8_t *sparam;
    int32_t iparam;
    int32_t iparam_len;
    va_list vl;
    va_start(vl, fmt);

    vosSemWait(gs.sendlock);
    vhalSerialWrite(gs.serial,"AT",2);
    printf("->: AT");
    vhalSerialWrite(gs.serial,cmd->body,cmd->len);
    printf("%s",cmd->body);
    while(*fmt){
        switch(*fmt){
            case 'i':
                //number
                iparam = va_arg(vl, int32_t);
                iparam_len = modp_itoa10(iparam,_strbuf);
                vhalSerialWrite(gs.serial,_strbuf,iparam_len);
                _strbuf[iparam_len]=0;
                printf("%s",_strbuf);
                break;
            case 's':
                sparam = va_arg(vl, uint8_t *);
                iparam_len = va_arg(vl,int32_t *);
                vhalSerialWrite(gs.serial,sparam,iparam_len);
#if defined(QUECTEL_BG96_DEBUG)
                for(iparam=0;iparam<iparam_len;iparam++) printf("%c",sparam[iparam]);
#endif
                break;
            default:
                vhalSerialWrite(gs.serial,fmt,1);
                printf("%c",*fmt);
        }
        fmt++;
    }
    vhalSerialWrite(gs.serial,"\r",1);
    printf("\n");
    vosSemSignal(gs.sendlock);
    va_end(vl);
}

/**
 * @brief Configure basic parameters for startup
 *
 * Disables echo, set CMEE to 2, set urcs and displays info about firmware
 *
 * @return 0 on failure
 */
int _gs_config0(){
    //clean serial
    
    //disable echo
    vhalSerialWrite(gs.serial,"ATE0\r\n",6);
    if(!_gs_wait_for_ok(500)) return 0;

    //full error messages
    _gs_send_at(GS_CMD_CMEE,"=i",2);
    if(!_gs_wait_for_ok(500)) return 0;
    
    //enable urc about network statu
    _gs_send_at(GS_CMD_CREG,"=i",2);
    if(!_gs_wait_for_ok(500)) return 0;
    
    //display product ID
    vhalSerialWrite(gs.serial,"ATI\r\n",5);
    if(!_gs_wait_for_ok(500)) return 0;

    //timezone update
    vhalSerialWrite(gs.serial,"AT+CTZU=1\r\n",11);
    if(!_gs_wait_for_ok(500)) return 0;

    // //display app versions
    // vhalSerialWrite(gs.serial,"AT+QAPPVER\r\n",12);
    // if(!_gs_wait_for_ok(500)) return 0;
   
    vosThSleep(TIME_U(1000,MILLIS)); 
    gs.initialized=1;
    return 1;
}

/**
 * @brief Check if a command response in gs.buffer is actually valid
 *
 * Validity is checked by making sure that the command respons is followed by ": "
 *
 * @param[in] cmd the command structure to check against
 *
 * @return the position of command arguments in gs.buffer, 0 on failure
 */
int _gs_valid_command_response(GSCmd *cmd){
    if (gs.buffer[cmd->len]==':' && gs.buffer[cmd->len+1]==' ' && gs.bytes>=cmd->len+2) {
        //valid
        return cmd->len+2;
    }
    return 0;
}

/**
 * @brief Handle received URC (unsolicited result code)
 *
 * Handled urcs are: CIEV, CREG, UUPSDA, UUSOCL, UUSORD, UUSORF.
 * In case of socket related URC, this function signals the socket
 * event semaphore to wake up threads suspended on a socket call.
 *
 * @param[in] cmd the GSCmd structure of the URC
 */
void _gs_handle_urc(GSCmd *cmd){
    int32_t p0,p1,p2,p3,nargs;
    uint8_t *s0,*s1,*s2,*s3;
    int p = _gs_valid_command_response(cmd);
    uint8_t *buf  = gs.buffer+p;
    uint8_t *ebuf = gs.buffer+gs.bytes;
    GSocket *sock;
    if(!p) return;

    switch(cmd->id){
        case GS_CMD_QIOPEN:
        case GS_CMD_QSSLOPEN:
            nargs = _gs_parse_command_arguments(buf,ebuf,"ii",&p0,&p1);
            if(nargs!=2) goto exit_err;
            if (p1==0) {
                //socket opened!
                _gs_socket_opened(p0,1);
            } else {
                //socket error!
                _gs_socket_opened(p0,0);
            }
            break;
        case GS_CMD_QIURC:
        case GS_CMD_QSSLURC:
            nargs = _gs_parse_command_arguments(buf,ebuf,"s",&s0,&p0);
            if(nargs!=1) goto exit_err;
            if (p0==8 && memcmp(s0,"\"closed\"",p0)==0) {
                //socket closed!
                _gs_parse_command_arguments(buf,ebuf,"si",&s0,&p0,&p1);
                _gs_socket_closing(p1);
            } else if(p0==6 & memcmp(s0,"\"recv\"",p0)==0){
                //data ready!
                _gs_parse_command_arguments(buf,ebuf,"si",&s0,&p0,&p1);
                _gs_socket_pending(p1);
            } else if(p0==8 & memcmp(s0,"\"dnsgip\"",p0)==0){
                //dns ready!
                _gs_parse_command_arguments(buf,ebuf,"ss",&s0,&p0,&s1,&p1);
                if (s1[0]=='0'){
                    //ok...get ipcount
                    _gs_parse_command_arguments(buf,ebuf,"ssi",&s0,&p0,&s1,&p1,&p2);
                    printf("Set dns count %i\n",p2);
                    gs.dns_count = p2;
                } else {
                    gs.dns_count--;
                    if (s1[0]=='"') {
                        //it's an IP
                        printf("DNS %x -> %x of %i\n",s1,gs.dnsaddr,p1-2); 
                        memcpy(gs.dnsaddr,s1+1,p1-2); //rermove quotes around ip
                        gs.dnsaddrlen=p1-2;
                    } else {
                        // errors or no idea
                        gs.dnsaddrlen=0;
                        gs.dns_count=0; //exit
                    }
                    printf("DNS COUNT %i\n",gs.dns_count); 
                    if(!gs.dns_count) gs.dns_ready=1;
                }

            } else {
                //socket error!
                _gs_socket_opened(p0,0);
            }
            break;

        default:
            printf("Unhandled URC %i\n",cmd->id);
    }

exit_ok:
    return;

exit_err:
    ;
    printf("Error parsing arguments for %i\n",cmd->id);
    return;
}

/**
 * @brief Wait for a slot to be available and acquires it
 *
 * A slot (or actually the slot, since in this implementation it is unique) is a structure holding information about the last issued command.
 * It also contains a buffer to hold the command response. Such buffer can be passed as an argument or (by passing NULL and a size) allocated by
 * the driver. In this case it will be deallocated on slot release. Acquiring a slot is a blocking operation and no other thread can access the serial port
 * until the slot is released.
 *
 * @param[i] cmd_id   the command identifier for the slot
 * @param[i] respbuf  a buffer sufficiently sized to hold the command response or NULL if such memory must be allocated by the driver
 * @param[i] max_size the size of respbuf. If 0 and respbuf is NULL, no memory is allocated. If positive and respbuf is NULL, memory is allocated by the driver
 * @param[i] timeout  the number of milliseconds before declaring the slot timed out
 * @param[i] nparams  the number of command response lines expected (0 or 1 in this implementation)
 *
 * @return a pointer to the acquired slot
 */
GSSlot *_gs_acquire_slot(int cmd_id, uint8_t *respbuf, int max_size, int timeout, int nparams){
    vosSemWait(gs.slotlock);
    gslot.cmd = GS_GET_CMD(cmd_id);
    gslot.stime = vosMillis();
    gslot.timeout = timeout;
    gslot.has_params = nparams;
    if(!respbuf){
        if(max_size){
            gslot.resp = gc_malloc(max_size);
            gslot.eresp = gslot.resp;
        } else {
            gslot.resp = gslot.eresp = NULL;
        }
        gslot.allocated = 1;
        gslot.max_size = max_size;
    } else {
        gslot.resp = gslot.eresp =respbuf;
        gslot.max_size = max_size;
        gslot.allocated = 0;
    }

    gs.slot = &gslot;
    return &gslot;
}

/**
 * @brief Wait until the main thread signal of slot completion
 */
void _gs_wait_for_slot(){
    vosSemWait(gs.slotdone);
}

/**
 * @brief Wait until the main thread signals the slot entering special mode (see +USOSECMNG)
 *
 * Special mode is requested by some commands. They do not return a response until some other lines of text
 * are sent after the AT command. textlen bytes are sent from text automatically before returning.
 * The calling thread must still wait for slot completion after this function returns.
 * This function can fail if the main thread does not signal spaecial mode after 10 seconds.
 *
 *
 * @param[in] text      the buffer to send
 * @param[in] textlen   the number of bytes to send
 *
 * @return 0 on success
 */
int _gs_wait_for_slot_mode(uint8_t *text, int32_t textlen){
    //can be polled!
    int cnt =0;
    printf("Waiting for mode\n");
    // vhalSerialWrite(gs.serial,">",1);
    while(gs.mode==GS_MODE_NORMAL &&cnt<100){ //after 10 seconds, timeout
        vosThSleep(TIME_U(100,MILLIS));
        cnt++;
    }

    if(gs.mode!=GS_MODE_PROMPT) return -1;
    printf("Slot wait mode\n");
    printf("-->%s\n",text);

    while(textlen>0){
        cnt = MIN(64,textlen);
        printf("Sending %i\n",cnt);
        cnt = vhalSerialWrite(gs.serial,text,cnt);
        printf("Sent %i\n",cnt);
        textlen-=cnt;
        text+=cnt;
        printf("Remaining %i\n",textlen);
    }
    gs.mode=GS_MODE_NORMAL; //back to normal mode

    return 0;
}

int _gs_wait_for_buffer_mode(){
    //can be polled!
    int cnt =0;
    printf("Waiting for buffer mode\n");
    while(gs.mode!=GS_MODE_BUFFER &&cnt<100){ //after 10 seconds, timeout
        vosThSleep(TIME_U(100,MILLIS));
        cnt++;
    }

    return gs.mode==GS_MODE_BUFFER;
}
int _gs_exit_from_buffer_mode_w(uint8_t* buf, int len){

        
    if(buf && len) {
        vhalSerialWrite(gs.serial,buf,len);
    }
    gs.mode = GS_MODE_NORMAL;
    vosSemSignal(gs.bufmode);
    return 0;
}

int _gs_exit_from_buffer_mode_r(uint8_t* buf, int len, int max, GSocket* sock){

        
    if (max<len) len=max;
    if(buf && len) {
        printf("bmode read %i\n",len);
        vhalSerialRead(gs.serial,buf,len);
        if(sock){
            int pos;
            while(max>len){
                pos = (sock->head+sock->len)%MAX_SOCK_RX_BUF;
                printf("bmode read 1 at %i/%i pos %i\n",sock->head,sock->len,pos);
                vhalSerialRead(gs.serial,sock->rxbuf+pos,1);
                sock->len++;
                max--;
            }
        } else {
            while(max>len){
                //skip up to max
                uint8_t dummy;
                vhalSerialRead(gs.serial,&dummy,1);
                max--;
            }
        }
    }
    gs.mode = GS_MODE_NORMAL;
    vosSemSignal(gs.bufmode);
    return 0;
}

/**
 * @brief Release an acquired slot
 *
 * Deallocate slot memory if needed
 *
 * @param[in] slot the slot to release
 */
void _gs_release_slot(GSSlot *slot){
    if (slot->allocated && slot->resp) gc_free(slot->resp);
    memset(slot,0,sizeof(GSSlot));
    vosSemSignal(gs.slotlock);
}

/**
 * @brief Signal the current slot as ok
 */
void _gs_slot_ok(){
    printf("ok slot %s\n",gs.slot->cmd->body);
    gs.slot->err = 0;
    gs.slot = NULL;
    vosSemSignal(gs.slotdone);
}

/**
 * @brief Signal the current slot as error
 */
void _gs_slot_error(){
    printf("error slot %s\n",gs.slot->cmd->body);
    gs.slot->err = 2;
    gs.slot = NULL;
    vosSemSignal(gs.slotdone);
}

/**
 * @brief Signal the current slot as timed out
 */
void _gs_slot_timeout(){
    printf("timeout slot %s\n",gs.slot->cmd->body);
    gs.slot->err = GS_ERR_TIMEOUT;
    gs.slot = NULL;
    vosSemSignal(gs.slotdone);
}


/**
 * @brief Transfer the command response in gs.buffer to the slot memory
 *
 * @param[in] cmd the command to transfer
 */
void _gs_slot_params(GSCmd *cmd){
    //copy params to slot
    if (!gs.slot->resp) return;
    if(cmd->response_type == GS_RES_STR || cmd->response_type== GS_RES_STR_OK) {
        int csize = (gs.slot->max_size<gs.bytes) ? gs.slot->max_size:gs.bytes;
        memcpy(gs.slot->resp,gs.buffer,csize);
        gs.slot->eresp = gs.slot->resp+csize;
    } else {
        if (!_gs_valid_command_response(cmd)) return;
        int psize = gs.bytes-cmd->len-2;
        int csize = (gs.slot->max_size<psize) ? gs.slot->max_size:psize;
        memcpy(gs.slot->resp,gs.buffer+cmd->len+2,csize);
        gs.slot->eresp = gs.slot->resp+csize;
    }
    gs.slot->params++;
}

/**
 * @brief Main thread loop
 *
 * Exit when the driver is deinitialized
 *
 * @param[i] args thread arguments
 */
void _gs_loop(void *args){
    (void)args;
    GSCmd *cmd;
    printf("_gs_loop started\n");
    while (gs.initialized){
        // printf("looping\n");
        if(gs.mode == GS_MODE_NORMAL){
            if(_gs_readline(100)<=3){
                if(gs.bytes>=1 && gs.buffer[0]=='>' && gs.slot && (gs.slot->cmd->id == GS_CMD_QISEND || gs.slot->cmd->id == GS_CMD_QSSLSEND)){
                    //only enter in prompt mode if the current slot is for QISEND to avoid locks
                    printf("GOT PROMPT!\n");
                    gs.mode = GS_MODE_PROMPT;
                    continue;
                }
                //no line
                if (gs.slot) {
                    if (gs.slot->timeout && (vosMillis()-gs.slot->stime)>gs.slot->timeout){
                        //slot timed out
                        printf("slot timeout\n");
                        _gs_slot_timeout();
                    }
                }
                continue;
            }
            cmd = _gs_parse_command_response();
            if (gs.slot) {
                //we have a slot
                if (cmd) {
                    printf("CMD %x %x\n",cmd,gs.slot->cmd);
                    //we parsed a command
                    if(cmd == gs.slot->cmd) {
                        //we parsed the response to slot
                        if (gs.slot->has_params){
                            printf("filling slot params for %s\n",cmd->body);
                            _gs_slot_params(cmd);
                            if(cmd->id == GS_CMD_QIRD || cmd->id == GS_CMD_QSSLRECV) {
                                //it's a QIRD response, enter read buffer mode!
                                //unless it's just a check 
                                gs.mode = GS_MODE_BUFFER;
                            }
                        } else {
                            printf("Unexpected params for slot\n");
                        }
                    } else if (cmd->urc) {
                        //we parsed a urc
                        printf("Handling urc %s in a slot\n",cmd->body);
                        _gs_handle_urc(cmd);
                    }
                } else {
                    //we don't have a command
                    if (_gs_check_ok()){
                        //we got an OK...is it for the current slot?
                        if (gs.slot->has_params == gs.slot->params){
                            _gs_slot_ok();
                        } else {
                            printf("Unexpected OK %s %i %i\n",gs.slot->cmd->body,gs.slot->params,gs.slot->has_params);
                        }
                    } else if (_gs_check_error()){
                        _gs_slot_error();
                    } else if(gs.slot->cmd->response_type == GS_RES_STR) {
                        //the command behaves differently
                        printf("filling slot params for GS_RES_STR\n");
                        _gs_slot_params(gs.slot->cmd);
                        _gs_slot_ok();
                    } else if(gs.slot->cmd->response_type == GS_RES_STR_OK) {
                        //the command behaves differently
                        printf("filling slot params for GS_RES_STR_OK\n");
                        _gs_slot_params(gs.slot->cmd);
                    } else{
                        if (gs.slot->cmd->id==GS_CMD_QFUPL && memcmp(gs.buffer,"CONNECT",7)==0) {
                            // go in buffer mode
                            gs.mode = GS_MODE_BUFFER;
                        } else {
                            printf("Unexpected line\n");
                        }
                    }
                }
            } else {
                // we have no slot
                if  (cmd) {
                    //we have a command
                    if(cmd->urc) {
                        printf("Handling urc %s out of slot\n",cmd->body);
                        _gs_handle_urc(cmd);
                    } else {
                        printf("Don't know what to do with %s\n",cmd->body);
                    }
                } else {
                    // we have no command
                    printf("Unknown line out of slot\n");
                }
            }
        } else  if(gs.mode == GS_MODE_PROMPT) {
            //// PROMPT MODE 
            //Prompt mode is used for USECMNG (implemented) and DWNFILE (not implemented)
            //If needed, logic for prompt mode goes here
            int ss;
            for(ss=0;ss<40;ss++){
                //avoid locking, max time spendable in prompt mode = 20s
                vosThSleep(TIME_U(500,MILLIS));
                if (gs.mode!=GS_MODE_PROMPT) break;
            }
            gs.mode = GS_MODE_NORMAL;

        } else {
            //standby here!
            printf("Entering buffer mode\n");
            vosSemWait(gs.bufmode);
            printf("Exited buffer mode\n");
        }
    }

}

/////////////////////SOCKET HANDLING
//
// The following functions implemented BSD compatible sockets on top of AT commands.
// CNatives and utilities functions follow the convention above


/**
 * \mainpage Socket Management
 * 
 *  GSocket structure contains two semaphores, one to gain exclusive access to the structure (socklock) the other
 *  to signal event to threads suspended on a socket receive. Since sockets can be closed remotely, GSocket also has a flag (to_be_closed) 
 *  indicating such event.
 *
 *  The id of a socket (index in the socket list) is assigned by the +USOCR command. If a previously created GSocket with the same id
 *  returned by +USOCR has not been properly closed, the creation of the corresponding new GSocket fails until correct closing. This prevents memory leaks
 *  and the possibility of having in Python two sockets instances with the same id (one valid and one invalid).
 *
 *  The event about pending bytes in the receive queue is signaled by the module with one or more URCs. 
 *  The URC handling routine signal the appropriate socket on the rx semaphore.
 *  There is no check about the status of the socket (this would have involved a more complicated coordination between he main thread
 *  and sockets). 
 *
 *  CNatives that implements the various form of recv suspend themselves on the rx semaphore when the module AT command 
 *  (+USORD or +USORF) return 0 available bytes. However, since the urc handling routine may signal pending bytes repeatedly, it can happen that
 *  even at 0 available bytes, rx semaphore won't suspend. This is not a big issue, since the additional (and unneeded) AT command executions are quite fast.
 *
 *
 */





/**
 * @brief creates a new socket with proto
 *
 * Sockets are numbered from 0 to MAX_SOCKS by the g350 module on creation, therefore
 * creating a GSocket means initializing the structure with default values.
 *
 * @param[in] proto the proto type (6=TCP, 17=UDP)
 *
 * @return the socket id or negatve in case of error
 */
int _gs_socket_new(int proto, int secure){
    GSocket *sock;
    int res= -1;
    int i = 0;

    //TODO: protect with global semaphore
    for(i=0;i<MAX_SOCKS;i++) {
        sock = &gs_sockets[i];
        // vosSemWait(sock->lock);
        if(!sock->acquired) {

            if (sock->to_be_closed) {
                _gs_socket_close(i);
            }
            sock->acquired      = 1;
            sock->to_be_closed  = 0;
            sock->connected     = 0;
            sock->timeout       = 0;
            sock->bound         = 0;
            sock->secure        = secure;
            sock->proto         = proto;
            sock->head          = 0;
            sock->len           = 0;
            res = i;
            // vosSemSignal(sock->lock);
            break;
        }
        // vosSemSignal(sock->lock);
    }

    return res;
}

int _gs_file_delete(uint8_t* filename, int namelen){
    GSSlot *slot;
    int res = 0;

    slot = _gs_acquire_slot(GS_CMD_QFDEL,NULL,0,GS_TIMEOUT,0);
    _gs_send_at(GS_CMD_QFDEL,"=\"s\"",filename,namelen);
    _gs_wait_for_slot();
    res = slot->err;
    _gs_release_slot(slot);

    return res;
}

int _gs_file_upload(uint8_t* filename, int namelen, uint8_t* content, int len){
    GSSlot *slot;
    int res = 0;
    int snt = 0;
    int tsnd = 0;

    slot = _gs_acquire_slot(GS_CMD_QFUPL,NULL,64,GS_TIMEOUT*60,1);
    _gs_send_at(GS_CMD_QFUPL,"=\"s\",i,5,0",filename,namelen,len);
    printf("WAIT\n");
    _gs_wait_for_buffer_mode();
    printf("WRITE\n");
    _gs_exit_from_buffer_mode_w(content, len);
    printf("SLOT\n");
    _gs_wait_for_slot();
    res = slot->err;
    _gs_release_slot(slot);
   return res;
}

uint8_t f_cacert[12]  = "cacert#.pem";
uint8_t f_clicert[12] = "clicrt#.pem";
uint8_t f_prvkey[12]  = "prvkey#.pem";


int _gs_ssl_cfg(int op, int ctx, int val) {
    GSSlot *slot;
    int res = 0;

    //WARNING: names of certificates are global!

    slot = _gs_acquire_slot(GS_CMD_QSSLCFG,NULL,0,GS_TIMEOUT*5,0);
    switch(op) {
        case 0:
            _gs_send_at(GS_CMD_QSSLCFG,"=\"s\",i,3","sslversion",10,ctx);  //select TLS 1.2 only
            break;
        case 1:
            _gs_send_at(GS_CMD_QSSLCFG,"=\"s\",i,0x0035,0x002f,0x003d,0xc013,0xc014,0xc027,0xc028,0xc02f","ciphersuite",11,ctx);  //select all secure ciphersuites
            break;
        case 2:
            _gs_send_at(GS_CMD_QSSLCFG,"=\"s\",i,\"s\"","cacert",6,ctx,f_cacert,11);  //select cacert
            break;
        case 3:
            _gs_send_at(GS_CMD_QSSLCFG,"=\"s\",i,\"s\"","clientcert",10,ctx,f_clicert,11);  //select clicert
            break;
        case 4:
            _gs_send_at(GS_CMD_QSSLCFG,"=\"s\",i,\"s\"","clientkey",9,ctx,f_prvkey,11);  //select prvkey
            break;
        case 5:
            _gs_send_at(GS_CMD_QSSLCFG,"=\"s\",i,i","seclevel",8,ctx,val);  //select prvkey
            break;
        case 6:
            _gs_send_at(GS_CMD_QSSLCFG,"=\"s\",i,i","ignorelocaltime",15,ctx,val);  //select date check
            break;
        case 7:
            _gs_send_at(GS_CMD_QSSLCFG,"=\"s\",i,i","negotiatetime",13,ctx,val);  //select timeout
            break;
    }
    _gs_wait_for_slot();
    res = slot->err;
    _gs_release_slot(slot);
    return res;
}

int _gs_socket_tls(int id, uint8_t* cacert, int cacertlen, uint8_t* clicert, int clicertlen, uint8_t* pvkey, int pvkeylen, int authmode){
    GSocket *sock;
    GSSlot *slot;
    int res = 0;
    int ctx = id;
    sock = &gs_sockets[id];
    
    //SSL CTX = socket id
    //let's upload files..


    vosSemWait(sock->lock);

    res+=_gs_ssl_cfg(0,ctx,3); //TLS 1.2
    res+=_gs_ssl_cfg(1,ctx,0); //all ciphers

    if(cacert && cacertlen) {
        f_cacert[6]='0'+id;
        _gs_file_delete(f_cacert,11);
        res+=_gs_file_upload(f_cacert,11,cacert,cacertlen);
        res+=_gs_ssl_cfg(2,ctx,0);
    }
    if(clicert && clicertlen) {
        f_clicert[6]='0'+id;
        _gs_file_delete(f_clicert,11);
        res+=_gs_file_upload(f_clicert,11,clicert,clicertlen);
        res+=_gs_ssl_cfg(3,ctx,0);
    }
    if(pvkey && pvkeylen) {
        f_prvkey[6]='0'+id;
        _gs_file_delete(f_prvkey,11);
        res+=_gs_file_upload(f_prvkey,11,pvkey,pvkeylen);
        res+=_gs_ssl_cfg(4,ctx,0);
    }

    res+=_gs_ssl_cfg(5,ctx,authmode); //0 none, 1 server, 2 server+client
    res+=_gs_ssl_cfg(6,ctx,0); //set validity check
    res+=_gs_ssl_cfg(1,ctx,0); //set secure ciphersuites
    vosSemSignal(sock->lock);

    return res;
}

int _gs_socket_opened(int id, int success){
    GSocket *sock;
    sock = &gs_sockets[id];
    if (success) sock->connected=1;
    else sock->connected=2;
    return 0;
}

int _gs_socket_bind(int id, NetAddress *addr){
    int res = -1;
    int timeout = 160000; //150s timeout for URC
    GSocket *sock;
    GSSlot *slot;
    sock = &gs_sockets[id];
    
    vosSemWait(sock->lock);

    slot = _gs_acquire_slot(GS_CMD_QIOPEN,NULL,0,GS_TIMEOUT*60*3,0);
    if (sock->proto==17){
        _gs_send_at(GS_CMD_QIOPEN,"=i,i,\"UDP SERVICE\",\"127.0.0.1\",0,i,0",GS_PROFILE,id,OAL_GET_NETPORT(addr->port));
    }
    _gs_wait_for_slot();
    if (slot->err) {
        res=-1;
    }
    _gs_release_slot(slot);
    if (res) {
        //release socket
        sock->acquired = 0;
    }

    vosSemSignal(sock->lock);

    while(timeout>0){
        vosThSleep(TIME_U(100,MILLIS));
        timeout-=100;
        vosSemWait(sock->lock);
        if (sock->connected) {
           if(sock->connected==2) {
            res=-2;
           } else {
            res = 0;
           }
        }
        vosSemSignal(sock->lock);
        if(!res||res==-2) break;
    }

    if(res) {
        //oops, timeout or error
        vosSemWait(sock->lock);
        sock->acquired=0;
        vosSemSignal(sock->lock);
    } else {
        sock->bound = 1;
    }
    return res;

}

int _gs_socket_connect(int id, NetAddress *addr){
    uint8_t saddr[16];
    uint32_t saddrlen;
    int res = -1;
    int timeout = 160000; //150s timeout for URC
    saddrlen = _gs_socket_addr(addr,saddr);
    GSocket *sock;
    GSSlot *slot;
    sock = &gs_sockets[id];
    vosSemWait(sock->lock);

    if (sock->secure) {
        slot = _gs_acquire_slot(GS_CMD_QSSLOPEN,NULL,0,GS_TIMEOUT*60*3,0);
        if (sock->proto==6){
            _gs_send_at(GS_CMD_QSSLOPEN,"=i,i,i,\"s\",i",GS_PROFILE,id,id,saddr,saddrlen,OAL_GET_NETPORT(addr->port));
        } 
        //NO DTLS!!
        // else {
        //     _gs_send_at(GS_CMD_QIOPEN,"=i,i,i,\"UDP\",\"s\",i",GS_PROFILE,id,saddr,saddrlen,OAL_GET_NETPORT(addr->port));
        // }
    } else {
        slot = _gs_acquire_slot(GS_CMD_QIOPEN,NULL,0,GS_TIMEOUT*60*3,0);
        if (sock->proto==6){
            _gs_send_at(GS_CMD_QIOPEN,"=i,i,\"TCP\",\"s\",i,0,0",GS_PROFILE,id,saddr,saddrlen,OAL_GET_NETPORT(addr->port));
        } else {
            //udp
            _gs_send_at(GS_CMD_QIOPEN,"=i,i,\"UDP\",\"s\",i,0,0",GS_PROFILE,id,saddr,saddrlen,OAL_GET_NETPORT(addr->port));
        }
    }
    _gs_wait_for_slot();
    if (slot->err) {
        res=-1;
    }
    _gs_release_slot(slot);
    if (res) {
        //release socket
        sock->acquired = 0;
    }

    vosSemSignal(sock->lock);

    while(timeout>0){
        vosThSleep(TIME_U(100,MILLIS));
        timeout-=100;
        vosSemWait(sock->lock);
        if (sock->connected) {
           if(sock->connected==2) {
            res=-2;
           } else {
            res = 0;
           }
        }
        vosSemSignal(sock->lock);
        if(!res||res==-2) break;
    }

    if(res) {
        //oops, timeout or error
        vosSemWait(sock->lock);
        sock->acquired=0;
        vosSemSignal(sock->lock);
    }
    return res;
}

/**
 * @brief retrieve the socket with a specific id if it exists
 *
 * @param[in] id    the socket id
 *
 * @return the socket or NULL on failure
 */
GSocket *_gs_socket_get(int id){
    GSocket *sock, *res=NULL;

    sock = &gs_sockets[id];
    vosSemWait(sock->lock);

    if(sock->acquired)  res = sock;
    
    vosSemSignal(sock->lock);
    return res;
}

/**
 * @brief 
 *
 * @param id
 */
int _gs_socket_close(int id){
    GSocket *sock;
    GSSlot *slot;
    int res = 0;
    sock = &gs_sockets[id];


    vosSemWait(sock->lock);
    if (sock->secure) {
        slot = _gs_acquire_slot(GS_CMD_QSSLCLOSE,NULL,0,GS_TIMEOUT*10,0);
        _gs_send_at(GS_CMD_QSSLCLOSE,"=i,10",id);
    } else {
        slot = _gs_acquire_slot(GS_CMD_QICLOSE,NULL,0,GS_TIMEOUT*10,0);
        _gs_send_at(GS_CMD_QICLOSE,"=i,10",id);
    }
    _gs_wait_for_slot();
    if (slot->err) {
        res=-1;
    }
    _gs_release_slot(slot);
    //regardless of the error, close

    //unlock sockets waiting on rx
    sock->acquired = 0;
    vosSemSignal(sock->rx);
    vosSemSignal(sock->lock);
    return res;
}

int _gs_socket_send(int id, uint8_t* buf, int len) {
    int res=len;
    GSSlot *slot;
    GSocket *sock;
    sock = &gs_sockets[id];


    vosSemWait(sock->lock);
    if (sock->to_be_closed) {
        res=-1;
    } else {
        if (sock->secure){
            slot = _gs_acquire_slot(GS_CMD_QSSLSEND,NULL,32,GS_TIMEOUT*10,1);
            _gs_send_at(GS_CMD_QSSLSEND,"=i,i",id,len);
        } else {
            slot = _gs_acquire_slot(GS_CMD_QISEND,NULL,32,GS_TIMEOUT*10,1);
            _gs_send_at(GS_CMD_QISEND,"=i,i",id,len);
        }
        res = _gs_wait_for_slot_mode(buf,len);
        if(res) {
            //ouch!
        } else {
            res=len;
        }
        _gs_wait_for_slot();
        if(slot->err) {
           res = -1; 
        } else {
            //check resp
            if (memcmp(slot->resp,"SEND FAIL",9)==0) {
                //buffer full!
                res = 0;
            }
        }
        _gs_release_slot(slot);
    }
    vosSemSignal(sock->lock);

    return res;
}

int _gs_socket_sendto(int id, uint8_t* buf, int len, NetAddress *addr) {
    int res=len;
    int saddrlen;
    GSSlot *slot;
    GSocket *sock;
    uint8_t remote_ip[16];
    sock = &gs_sockets[id];

    saddrlen = _gs_socket_addr(addr,remote_ip);

    vosSemWait(sock->lock);
    if (sock->to_be_closed) {
        res=-1;
    } else {
        slot = _gs_acquire_slot(GS_CMD_QISEND,NULL,32,GS_TIMEOUT*10,1);
        _gs_send_at(GS_CMD_QISEND,"=i,i,\"s\",i",id,len,remote_ip,saddrlen,OAL_GET_NETPORT(addr->port));
        res = _gs_wait_for_slot_mode(buf,len);
        if(res) {
            //ouch!
        } else {
            res=len;
        }
        _gs_wait_for_slot();
        if(slot->err) {
           res = -1; 
        } else {
            //check resp
            if (memcmp(slot->resp,"SEND FAIL",9)==0) {
                //buffer full!
                res = 0;
            }
        }
        _gs_release_slot(slot);
    }
    vosSemSignal(sock->lock);

    return res;
}

int _gs_socket_recvfrom(int id, uint8_t* buf, int len, uint8_t* addr, int*addrlen, int*port){
    int trec=0;
    int res = len;
    int rd;
    int nargs;
    uint8_t *remote_ip;
    int remote_len;
    int remote_port;
    GSSlot *slot;
    GSocket *sock;

    sock = &gs_sockets[id];
    vosSemWait(sock->lock);
    if (sock->to_be_closed) {
        res=-1;
    } else {
        trec = MIN(MAX_SOCK_BUF,len);
        slot = _gs_acquire_slot(GS_CMD_QIRD,NULL,64,GS_TIMEOUT*10,1);
        _gs_send_at(GS_CMD_QIRD,"=i",id);
        if(!_gs_wait_for_buffer_mode()){
            //oops, timeout
            res = -1;
        }
        
        nargs = _gs_parse_command_arguments(slot->resp,slot->eresp,"isi",&rd,&remote_ip,&remote_len,&remote_port);
        printf("READ NARGS %i %i %i %i\n",nargs,rd,remote_len,remote_port);
        if(nargs==3){
            res = MIN(rd,len);
            memcpy(addr,remote_ip+1,MIN(15,(remote_len-2)));
            *addrlen = remote_len-2;
            *port = remote_port;
            _gs_exit_from_buffer_mode_r(buf,len,rd,NULL);
        } else {
            if (nargs>1 && rd==0 && remote_len==0) {
                //no data
                res=0;
                _gs_exit_from_buffer_mode_r(NULL,0,0,NULL);
            } else {
                res = -1;
                _gs_exit_from_buffer_mode_r(NULL,0,0,NULL);
            }
        }
        _gs_wait_for_slot();
        if (slot->err) {
            res= -1;
        }
        _gs_release_slot(slot);
    }
    vosSemSignal(sock->lock);
    if (res==0) {
        //empty buffer, wait with timeout
        printf("Waiting for rx\n");
        vosSemWaitTimeout(sock->rx,TIME_U(5000,MILLIS));
    }
    return res;
}

int _gs_sock_copy(int id, uint8_t* buf, int len){
    GSocket *sock;
    int i,rd;
    sock = &gs_sockets[id];
   
    printf("Sock copy\n");
    rd=0;
    if(sock->len>0){
        rd=MIN(sock->len,len);
        printf("COPY %i from %i to %i/%i\n",rd,sock->head,(sock->head+rd)%MAX_SOCK_RX_BUF,sock->len);
        for(i=0;i<rd;i++){
            *buf++=sock->rxbuf[sock->head];
            sock->head=(sock->head+1)%MAX_SOCK_RX_BUF;
        }
        sock->len-=rd;
        if (!sock->len){
            //reset buf
            sock->head = 0;
            sock->len = 0;
        }
    }
    return rd;
}

int _gs_socket_recv(int id, uint8_t* buf, int len){
    int trec=0;
    int res = len;
    int rd;
    GSSlot *slot;
    GSocket *sock;

    sock = &gs_sockets[id];
    vosSemWait(sock->lock);
    if (sock->to_be_closed) {
        res=-1;
    } else {
        //read from buffer
        rd = _gs_sock_copy(id, buf, len);
        buf+=rd;
        len-=rd;
        if(rd>0) {
            //skip command
            printf("Skip cmd\n");
            res=rd;
        }else {
            //read from slot
            trec = MIN(MAX_SOCK_BUF,len);
            if (sock->secure) {
                slot = _gs_acquire_slot(GS_CMD_QSSLRECV,NULL,64,GS_TIMEOUT*10,1);
                _gs_send_at(GS_CMD_QSSLRECV,"=i,i",id,trec);
            } else {
                slot = _gs_acquire_slot(GS_CMD_QIRD,NULL,64,GS_TIMEOUT*10,1);
                _gs_send_at(GS_CMD_QIRD,"=i,i",id,trec);
            }
            if(!_gs_wait_for_buffer_mode()){
                //oops, timeout
                res = -1;
            }

            if (_gs_parse_command_arguments(slot->resp,slot->eresp,"i",&rd)==1){
                res = MIN(trec,rd);
                _gs_exit_from_buffer_mode_r(buf,trec,rd,sock);
            } else {
                res = -1;
                _gs_exit_from_buffer_mode_r(NULL,0,0,NULL);
            }
            _gs_wait_for_slot();
            if (slot->err) {
                res= -1;
            }
            _gs_release_slot(slot);
        }
    }
    vosSemSignal(sock->lock);
    if (res==0) {
        //empty buffer, wait with timeout
        printf("Waiting for rx\n");
        vosSemWaitTimeout(sock->rx,TIME_U(5000,MILLIS));
    }
    return res;
}

int _gs_socket_available(int id){
    int trec=0;
    int res = 0;
    int total, rd, toberd;
    GSSlot *slot;
    GSocket *sock;

    sock = &gs_sockets[id];
    vosSemWait(sock->lock);
    if (sock->to_be_closed) {
        res=-1;
    } else {
    //read from buffer
    if (sock->len>0) res=sock->len;
    else {
        if (sock->secure) {
                slot = _gs_acquire_slot(GS_CMD_QSSLRECV,NULL,64,GS_TIMEOUT*10,1);
                _gs_send_at(GS_CMD_QSSLRECV,"=i,0",id);
            } else {
                slot = _gs_acquire_slot(GS_CMD_QIRD,NULL,64,GS_TIMEOUT*10,1);
                _gs_send_at(GS_CMD_QIRD,"=i,0",id);
            }
            if(!_gs_wait_for_buffer_mode()){
                //oops, timeout
                res = -1;
            } else {
                if (_gs_parse_command_arguments(slot->resp,slot->eresp,"iii",&total,&rd,&toberd)==3){
                    res = toberd;
                } else {
                    res = -1;
                }
                _gs_exit_from_buffer_mode_r(NULL,0,0,NULL);
            }
            _gs_wait_for_slot();
            if (slot->err) {
                res= -1;
            }
            _gs_release_slot(slot);
        }
    }
    vosSemSignal(sock->lock);
    return res;

}

void _gs_socket_closing(int id){
    GSocket *sock;
    sock = &gs_sockets[id];

    // vosSemWait(sock->lock);
    vosSemSignal(sock->rx);
    sock->to_be_closed=1;
    // vosSemSignal(sock->lock);
}

void _gs_socket_pending(int id){
    GSocket *sock;
    sock = &gs_sockets[id];
    vosSemSignal(sock->rx);
}



int _gs_resolve(uint8_t* url, int len, uint8_t* addr){
    int res = 0, cnt;
    GSSlot *slot;

    vosSemWait(gs.dnsmode);
    gs.dns_ready=0;
    slot = _gs_acquire_slot(GS_CMD_QIDNSGIP,NULL,0,GS_TIMEOUT*60,0);
    _gs_send_at(GS_CMD_QIDNSGIP,"=i,\"s\"",GS_PROFILE,url,len);
    _gs_wait_for_slot();
    if (slot->err) {
        res=-1;
    } 
    for(cnt=0;cnt<600;cnt++){
        //wait at most 60s to resolve
        vosThSleep(TIME_U(100,MILLIS));
        if(gs.dns_ready) break;
    }

    if(gs.dns_ready){
        res = gs.dnsaddrlen; //0 in case of error
        printf("copying from %x to %x %i bytes\n",gs.dnsaddr,addr,res);
        memcpy(addr,gs.dnsaddr,res);
    } else {
        res=-1;
    }
    _gs_release_slot(slot);
    vosSemSignal(gs.dnsmode);
    return res;

}


int _gs_socket_addr(NetAddress *addr, uint8_t *saddr) {
    uint8_t *buf = saddr;
    buf+=modp_itoa10(OAL_IP_AT(addr->ip, 0),buf);
    *buf++='.';
    buf+=modp_itoa10(OAL_IP_AT(addr->ip, 1),buf);
    *buf++='.';
    buf+=modp_itoa10(OAL_IP_AT(addr->ip, 2),buf);
    *buf++='.';
    buf+=modp_itoa10(OAL_IP_AT(addr->ip, 3),buf);
    return buf-saddr; 
}

// int _gs_socket_error( int sock){
//     int p0=-1,p1,p2;
//     GSSlot *slot;
//     slot = _gs_acquire_slot(GS_CMD_USOCTL,NULL,16,GS_TIMEOUT,1);
//     _gs_send_at(GS_CMD_USOCTL,"=i,i",sock,1);
//     _gs_wait_for_slot();
//     if (!slot->err) {
//         if(_gs_parse_command_arguments(slot->resp,slot->eresp,"iii",&p0,&p1,&p2)!=3) {
//             p0=-1;
//         } else {
//             p0=p2;
//         }
//     }
//     _gs_release_slot(slot);
//     return p0;
// }





/**
 * @brief Retrieve the list of operators with +COPS test command
 *
 * If successful, stores the retrieved operators with their parameters
 * in the global operator list (capped at MAX_OPS) and set their number (gsopn) accordingly
 *
 * @return 0 on success
 */
int _gs_list_operators(){
    GSSlot *slot;
    int err;
    slot = _gs_acquire_slot(GS_CMD_COPS,NULL,MAX_CMD,GS_TIMEOUT*60,1);
    _gs_send_at(GS_CMD_COPS,"=?");
    _gs_wait_for_slot();
    if (slot->err) {
        err = slot->err;
        _gs_release_slot(slot);
        return err;
    }
    uint8_t *buf = slot->resp;
    uint8_t nops =0, nres, nt=0;
    while(buf<slot->eresp){
        printf("start buf %x\n",*buf);
        if (!(*buf=='(' && *(buf+3)=='"')) break; //not a good record
        buf++; //skip (
        gsops[nops].type=*buf-'0';
        buf++; buf++; buf++; //skip ,"
        nt=0;
        while(*buf!='"'){
            gsops[nops].fmt_long[nt++]=*buf++;
        }
        gsops[nops].fmtl_l=nt;
        buf++; buf++; buf++; //skip ","
        nt=0;
        while(*buf!='"'){
            gsops[nops].fmt_short[nt++]=*buf++;
        }
        gsops[nops].fmts_l=nt;
        buf++; buf++; buf++; //skip ","
        nt=0;
        while(*buf!='"'){
            gsops[nops].fmt_code[nt++]=*buf++;
        }
        gsops[nops].fmtc_l=nt;
        //skip up to )
        while ((buf<slot->eresp) && (*buf!=')')) {
            printf("Skip %x\n",*buf);
            buf++;
        }
        buf++;
        if (*buf==',') buf++;  //skip the comma if present
        printf("Last %x\n",*buf);
        nops++;
        if (nops==MAX_OPS) break;
    }
    gsopn = nops;
    _gs_release_slot(slot);
    return 0;
}
int _gs_set_operator(uint8_t* operator, int oplen){
    GSSlot *slot;
    int err;
    slot = _gs_acquire_slot(GS_CMD_COPS,NULL,MAX_CMD,GS_TIMEOUT*60,0);
    _gs_send_at(GS_CMD_COPS,"=1,1,\"s\"",operator,oplen);
    _gs_wait_for_slot();
    err = slot->err;
    _gs_release_slot(slot);
    return err;
}

int _gs_check_network(){
    GSSlot *slot;
    int p0,p1,p2,l0,l1;
    uint8_t *s0,*s1;
    slot = _gs_acquire_slot(GS_CMD_CREG,NULL,64,GS_TIMEOUT*5,1);
    _gs_send_at(GS_CMD_CREG,"?");
    _gs_wait_for_slot();

    if(_gs_parse_command_arguments(slot->resp,slot->eresp,"iissi",&p0,&p1,&s0,&l0,&s1,&l1,&p2)!=5) {
        _gs_release_slot(slot);
        return 0;
    } else {
        memcpy(gs.lac,s0+1,l0-2);
        memcpy(gs.ci,s1+1,l1-2);
        gs.tech=p2;
    }
    _gs_release_slot(slot);
    if(p1==1 || p1==5) gs.registered = (p1==1) ? GS_REG_OK:GS_REG_ROAMING;
    return p1;
}

/**
 * @brief Generalize sending AT commands for activating/disactivating PSD
 *
 * @param[in] activate  1 for activation, 0 for deactivation
 *
 * @return 0 on failure
 */
int _gs_control_psd(int activate){
    GSSlot *slot;
    int res;
    activate = (activate) ? 1:0;
    if (activate) {
        slot = _gs_acquire_slot(GS_CMD_QIACT,NULL,0,GS_TIMEOUT*60*3,0);
        _gs_send_at(GS_CMD_QIACT,"=i",GS_PROFILE);
        _gs_wait_for_slot();
        res = !slot->err;
        _gs_release_slot(slot);
    } else {
        slot = _gs_acquire_slot(GS_CMD_QIDEACT,NULL,0,GS_TIMEOUT*60*3,0);
        _gs_send_at(GS_CMD_QIDEACT,"=i",GS_PROFILE);
        _gs_wait_for_slot();
        res = !slot->err;
        _gs_release_slot(slot);
    }
    return res;
}


/**
 * @brief Generalize sending AT commands to configure PSD
 *
 *
 * @return 0 on failure
 */
int _gs_configure_psd(uint8_t *apn, int apnlen, uint8_t *username, int ulen, uint8_t* pwd, int pwdlen, int auth){
    GSSlot *slot;
    slot = _gs_acquire_slot(GS_CMD_CGDCONT,NULL,0,GS_TIMEOUT,0);
    //set context id as undefined
    _gs_send_at(GS_CMD_CGDCONT,"=i",GS_PROFILE);
    _gs_wait_for_slot();
    int res = !slot->err;
    _gs_release_slot(slot);
    if (!res) return res;

    //configure TCP/IP PSD with IPV4IPV6
    slot = _gs_acquire_slot(GS_CMD_QICSGP,NULL,0,GS_TIMEOUT,0);
    _gs_send_at(GS_CMD_QICSGP,"=i,i,\"s\",\"s\",\"s\",i",GS_PROFILE,1,apn,apnlen,username,ulen,pwd,pwdlen,auth);
    _gs_wait_for_slot();
    res = !slot->err;
    _gs_release_slot(slot);

    return res;
}

int _gs_get_rtc(uint8_t* time){
    GSSlot *slot;
    uint8_t *s0;
    int l0;
    int res;
    slot = _gs_acquire_slot(GS_CMD_CCLK,NULL,32,GS_TIMEOUT,1);
    _gs_send_at(GS_CMD_CCLK,"?");
    _gs_wait_for_slot();
    res = !slot->err;
    if (res) {
        if(_gs_parse_command_arguments(slot->resp,slot->eresp,"s",&s0,&l0)!=1) {
            res = 0;
        } else {
            memcpy(time,s0+1,20);
        }
    }
    _gs_release_slot(slot);

    return res;
}


int _gs_rssi(){
    GSSlot *slot;
    int rssi=99, ber;
    slot = _gs_acquire_slot(GS_CMD_CSQ,NULL,32,GS_TIMEOUT,1);
    _gs_send_at(GS_CMD_CSQ,"");
    _gs_wait_for_slot();
    if (!slot->err) {
        if(_gs_parse_command_arguments(slot->resp,slot->eresp,"ii",&rssi,&ber)!=2) {
            rssi = 99;
        }
    }
    _gs_release_slot(slot);

    return rssi;
}
int _gs_attach(int attach){
    int res = 0;
    //Attach to GPRS
    GSSlot *slot;
    slot = _gs_acquire_slot(GS_CMD_CGATT,NULL,0,GS_TIMEOUT*60*3,0);
    _gs_send_at(GS_CMD_CGATT,"=i",attach);
    _gs_wait_for_slot();
    res = slot->err;
    _gs_release_slot(slot);
    return res;
}

int _gs_is_attached(){
    int status=0;
    //Attached to GPRS?
    GSSlot *slot;
    slot = _gs_acquire_slot(GS_CMD_CGATT,NULL,32,GS_TIMEOUT*60*3,1);
    _gs_send_at(GS_CMD_CGATT,"?");
    _gs_wait_for_slot();
    if (!slot->err) {
        if(_gs_parse_command_arguments(slot->resp,slot->eresp,"i",&status)!=1) {
            status = 0;
        }
    }
    _gs_release_slot(slot);
    gs.attached=status;
    return status;
}

int _gs_imei(uint8_t *imei){
    int res=-1;
    int l0;
    uint8_t *s0=NULL;
    GSSlot *slot;
    slot = _gs_acquire_slot(GS_CMD_GSN,NULL,32,GS_TIMEOUT,1);
    _gs_send_at(GS_CMD_GSN,"");
    _gs_wait_for_slot();
    if (!slot->err) {
        if(_gs_parse_command_arguments(slot->resp,slot->eresp,"s",&s0,&l0)!=1) {
            res = 0;
        } else {
            res = l0;
            if (s0) memcpy(imei,s0,MIN(16,l0));
        }
    }
    _gs_release_slot(slot);
    return res;
}
int _gs_iccid(uint8_t* iccid){
    int res=-1;
    int l0;
    uint8_t *s0=NULL;
    GSSlot *slot;
    slot = _gs_acquire_slot(GS_CMD_QCCID,NULL,32,GS_TIMEOUT,1);
    _gs_send_at(GS_CMD_QCCID,"");
    _gs_wait_for_slot();
    if (!slot->err) {
        if(_gs_parse_command_arguments(slot->resp,slot->eresp,"s",&s0,&l0)!=1) {
            res = 0;
        } else {
            res = l0;
            if (s0) memcpy(iccid,s0,MIN(22,l0));
        }
    }
    _gs_release_slot(slot);
    return res;

}

int _gs_dns(uint8_t*dns){
    int res=-1;
    int l0,p0;
    uint8_t *s0=NULL;
    GSSlot *slot;
    slot = _gs_acquire_slot(GS_CMD_QIDNSCFG,NULL,64,GS_TIMEOUT,1);
    _gs_send_at(GS_CMD_QIDNSCFG,"=?");
    _gs_wait_for_slot();
    if (!slot->err) {
        if(_gs_parse_command_arguments(slot->resp,slot->eresp,"i\"s\"",&p0,&s0,&l0)!=2) {
            res = 0;
        } else {
            res = l0;
            if (s0) memcpy(dns,s0,MIN(15,l0));
        }
    }
    _gs_release_slot(slot);
    return res;

}


int _gs_gnss_done(){
    int res=-1;
    GSSlot *slot;

    //turn off gnss
    slot = _gs_acquire_slot(GS_CMD_QGPSEND,NULL,64,GS_TIMEOUT,0);
    _gs_send_at(GS_CMD_QGPSEND,"");
    _gs_wait_for_slot();
    res = slot->err;
    _gs_release_slot(slot);

    return res;
}

int _gs_gnss_init(){
    int res=-1;
    GSSlot *slot;

    //disable nmea
    slot = _gs_acquire_slot(GS_CMD_QGPSCFG,NULL,64,GS_TIMEOUT,0);
    _gs_send_at(GS_CMD_QGPSCFG,"=s,i","\"nmeasrc\"",9,0);
    _gs_wait_for_slot();
    res = slot->err;
    _gs_release_slot(slot);
    if(res) return res;

    //enable all sources
    slot = _gs_acquire_slot(GS_CMD_QGPSCFG,NULL,64,GS_TIMEOUT,0);
    _gs_send_at(GS_CMD_QGPSCFG,"=s,i","\"gnssconfig\"",12,1);
    _gs_wait_for_slot();
    res = slot->err;
    _gs_release_slot(slot);
    if(res) return res;

    _gs_gnss_done();

    //turn on gnss
    slot = _gs_acquire_slot(GS_CMD_QGPS,NULL,64,GS_TIMEOUT,0);
    _gs_send_at(GS_CMD_QGPS,"=i",1);
    _gs_wait_for_slot();
    res = slot->err;
    _gs_release_slot(slot);


    return res;
}

#define GPS_NUM(x)  ((x)-'0')
#define GPS_NUM2(x,y)  ((GPS_NUM(x)*10)+(GPS_NUM(y)))


double _gs_float(uint8_t *s, int l){
    int intpart;
    int mantissa;
    int sign;
    int pow10;
    double res = 0;

    intpart = mantissa = sign = 0;
    pow10 = 1;

    if(l>0) {
        if(*s=='-') {
            sign=-1;
            s++;
            l--;
        } else sign=1;
        while(l && *s!='.') {
            intpart = intpart*10 +GPS_NUM(*s);
            s++;
            l--;
        }
        if(l){
            //we are at the dot
            s++;
            l--;
            while(l){
                mantissa = mantissa*10 +GPS_NUM(*s);
                pow10=pow10*10;
                s++;
                l--;
            }
        }
    }
    res = intpart + (double)mantissa/(double)pow10;
    return res*sign;
}

int _gs_gnss_loc(GNSSLoc *loc){
    int res=-1;
    GSSlot *slot;
    uint8_t *s0,*s1,*s2,*s3,*s4,*s5,*s6,*s7,*s8,*s9,*sa;
    int l0,l1,l2,l3,l4,l5,l6,l7,l8,l9,la;

    //turn off gnss
    slot = _gs_acquire_slot(GS_CMD_QGPSLOC,NULL,128,GS_TIMEOUT,1);
    _gs_send_at(GS_CMD_QGPSLOC,"=2");
    _gs_wait_for_slot();
    if(!slot->err) {
        if(_gs_parse_command_arguments(slot->resp,slot->eresp,"sssssssssss",
                    &s0,&l0,
                    &s1,&l1,
                    &s2,&l2,
                    &s3,&l3,
                    &s4,&l4,
                    &s5,&l5,
                    &s6,&l6,
                    &s7,&l7,
                    &s8,&l8,
                    &s9,&l9,
                    &sa,&la)!=11) {
            res = -1;
        } else {
            res = 0;
            //let's parse
            //s0 is utc time hhmmss.sss
            if(l0>=6){
                loc->hh=GPS_NUM2(s0[0],s0[1]);
                loc->mm=GPS_NUM2(s0[2],s0[3]);
                loc->ss=GPS_NUM2(s0[4],s0[5]);
            }
            loc->lat = _gs_float(s1,l1);
            loc->lon = _gs_float(s2,l2);
            loc->precision = _gs_float(s3,l3);
            loc->alt = _gs_float(s4,l4);
            loc->fix = (int) _gs_float(s5,l5);
            loc->cog = _gs_float(s6,l6);
            loc->cog = ((int)loc->cog)+((loc->cog-((int)loc->cog))*10/60);  // div by 0.6 to obtain degrees in th range 0-0.99999
            loc->speed = _gs_float(s7,l7);

            //date
            if(l9>=6) {
                loc->dd = GPS_NUM2(s9[0],s9[1]);
                loc->MM = GPS_NUM2(s9[2],s9[3]);
                loc->yy = GPS_NUM2(s9[4],s9[5]);
            }
            if(la>=2) {
                loc->nsat = GPS_NUM2(sa[0],sa[1]);
            }
        }
    }
    res = slot->err;
    _gs_release_slot(slot);

    return res;
}


#define RAT_GSM 0
#define RAT_M1 1
#define RAT_NB1 2

int _gs_set_rat(int rat, int band){
    int res=0;
    int prat,gsmband=0,m1band=0,nb1band=0;
    GSSlot *slot;

    if(rat<RAT_GSM || rat>RAT_NB1) return -1;
    printf("RAT %i %x\n",rat,band);

    if (rat==RAT_GSM) prat=1;
    else prat=3;


    // set scan mode
    slot = _gs_acquire_slot(GS_CMD_QCFG,NULL,0,GS_TIMEOUT,0);
    _gs_send_at(GS_CMD_QCFG,"=\"s\",i,1","nwscanmode",10,prat);
    _gs_wait_for_slot();
    res+=slot->err;
    _gs_release_slot(slot);

    if (rat==RAT_GSM){
        gsmband = band;
    } else if(rat==RAT_M1){
        m1band=band;
    } else if(rat==RAT_NB1){
        nb1band=band;
    }

    //set band
    slot = _gs_acquire_slot(GS_CMD_QCFG,NULL,0,GS_TIMEOUT,0);
    _gs_send_at(GS_CMD_QCFG,"=\"s\",i,i,i,1","band",4,gsmband,m1band,nb1band);
    _gs_wait_for_slot();
    res+=slot->err;
    _gs_release_slot(slot);

    //set iot mode
    if(rat!=RAT_GSM){
        int lte = (rat==RAT_M1) ? (0):(1);
        slot = _gs_acquire_slot(GS_CMD_QCFG,NULL,0,GS_TIMEOUT,0);
        _gs_send_at(GS_CMD_QCFG,"=\"s\",i,1","iotopmode",9,lte);
        _gs_wait_for_slot();
        res+=slot->err;
        _gs_release_slot(slot);
    }

    return res;
}

