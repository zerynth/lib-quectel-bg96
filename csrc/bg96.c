/**
 * @file bg96.c
 * @brief Driver for Quectel BG96 modules
 * @author Giacomo Baldi
 * @version 
 * @date 2019-26-09
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

#include "bg96.h"

#if 1
#define printf(...) vbl_printf_stdout(__VA_ARGS__)
#define print_buffer(bf, ln) 	for(uint8_t i = 0; i < ln; i++) { \
					printf("%c", bf[i]); \
				} \
				printf("\n"); 
#else
#define printf(...)
#define print_buffer(bf, ln)
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
void _gs_init(void)
{
    int i;
    if (!gs.initialized) {
        printf("Initializing GSM\n");
        for (i = 0; i < MAX_SOCKS; i++) {
            gs_sockets[i].lock = vosSemCreate(1);
            gs_sockets[i].rx = vosSemCreate(0);
        }
        memset(&gs, 0, sizeof(GStatus));
        gs.slotlock = vosSemCreate(1);
        gs.sendlock = vosSemCreate(1);
        gs.slotdone = vosSemCreate(0);
        gs.bufmode = vosSemCreate(0);
        gs.dnsmode = vosSemCreate(1);
        gs.selectlock = vosSemCreate(0);
        gs.pendingsms = 0;
        gs.initialized = 1;
        gs.talking = 0;
        gs.running = 0;
    }
    //TODO: regardless of initialized status, reset all sockets
}

/**
 * @brief Start modem loop and wait for running state
 *
 * @return 0 on success
 */
int _gs_start(void)
{
    int i;
    if (!gs.talking) {
        gs.talking = 1;
        for (i = 30; i > 0; --i) {
            printf("waiting modem loop %i\n",i);
            if (gs.running)
                break;
            vosThSleep(TIME_U(100, MILLIS));
        }
        if (i == 0)
            return GS_ERR_TIMEOUT;
    }
    if (!gs.running)
        return GS_ERR_INVALID;
    printf("started.\n");
    return GS_ERR_OK;
}

/**
 * @brief Stop modem loop and wait for idle state
 *
 * @return 0 on success
 */
int _gs_stop(void)
{
    int i;
    if (gs.talking) {
        gs.talking = 0;
        for (i = 50; i > 0; --i) {
            printf("waiting modem loop %i\n",i);
            if (!gs.running)
                break;
            vosThSleep(TIME_U(100, MILLIS));
        }
        if (i == 0)
            return GS_ERR_TIMEOUT;
    }
    if (gs.running)
        return GS_ERR_INVALID;
    printf("stopped.\n");
    return GS_ERR_OK;
}

/**
 * @brief Empty the serial receive buffer
 *
 * Use to discard old received data not pertaining to next command!
 */
void _gs_empty_rx(void)
{
    int bytes = vhalSerialAvailable(gs.serial);
    while (bytes > 0) {
        bytes = MIN(bytes, MAX_BUF - 1);
        vhalSerialRead(gs.serial, gs.buffer, bytes);
        //terminate for debugging!
        gs.buffer[bytes + 1] = 0;
        printf("re: %s\n", gs.buffer);
        // check if anything else has been received
        vosThSleep(TIME_U(10, MILLIS));
        bytes = vhalSerialAvailable(gs.serial);
    }
    gs.buffer[0] = 0;
    gs.bytes = 0;
}

/**
 * @brief Read a line from the module
 *
 * Lines are saved into gs.buffer and null terminated. The number of bytes read 
 * is saved in gs.bytes and returned. The timeout is implemented with a 50 milliseconds
 * polling strategy. TODO: change when the serial driver will support timeouts
 *
 * @param[in]   timeout     the number of milliseconds to wait for a line
 *
 * @return the number of bytes read or -1 on timeout
 */
int _gs_readline(int timeout)
{
    gs.bytes = 0;
    memset(gs.buffer, 0, 16);
    uint8_t* buf = gs.buffer;
    uint32_t tstart = vosMillis();
    while (gs.bytes < (MAX_BUF - 1)) {
        if (timeout > 0) {
            if ((vosMillis() - tstart) > timeout) {
                *buf = 0;
                return -1;
            }
            if (vhalSerialAvailable(gs.serial) > 0) {
                vhalSerialRead(gs.serial, buf, 1);
            } else {
                vosThSleep(TIME_U(50, MILLIS));
                continue;
            }
        } else {
            vhalSerialRead(gs.serial, buf, 1);
        }
        gs.bytes++;
        //        printf("->%i\n",gs.bytes);
        if (*buf++ == '\n')
            break;
    }
    //terminate for debugging!
    *buf = 0;
    printf("rl: %s", gs.buffer);
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
int _gs_read(int bytes)
{
    memset(gs.buffer, 0, 16);
    if (bytes <= 0)
        bytes = vhalSerialAvailable(gs.serial);
    gs.bytes = MIN(bytes, MAX_BUF - 1);
    vhalSerialRead(gs.serial, gs.buffer, gs.bytes);
    //terminate for debugging!
    gs.buffer[gs.bytes + 1] = 0;
    //printf("rn: %s\n",gs.buffer);
    return gs.bytes;
}

/**
 * @brief Checks if gs.buffer contains a valid "OK\r\n"
 *
 * @return 0 on failure
 */
int _gs_check_ok(void)
{
    return gs.bytes >= 4 && memcmp(gs.buffer, "OK\r\n", 4) == 0;
}
int _gs_check_rdy(void)
{
    return gs.bytes >= 5 && memcmp(gs.buffer, "RDY\r\n", 5) == 0;
}

/**
 * @brief Read lines from the module until a "OK" is received
 *
 * @param[in]   timeout     the number of milliseconds to wait for each line
 *
 * @return 0 on failure
 */
int _gs_wait_for_ok(int timeout)
{
    while (_gs_readline(timeout) >= 0) {
        if (_gs_check_ok()) {
            return 1;
        }
    }
    return 0;
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
int _gs_check_error(void)
{
    if (gs.bytes >= 12 && memcmp(gs.buffer, "+CME ERROR: ", 12) == 0) {
        int elen = MIN(gs.bytes - 12, MAX_ERR_LEN);
        memcpy(gs.errmsg, gs.buffer + 12, elen);
        gs.errlen = elen;
        return 1;
    } else if (gs.bytes >= 5 && memcmp(gs.buffer, "ERROR", 5) == 0) {
        gs.errlen = 0;
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
GSCmd* _gs_parse_command_response(void)
{
    int e0 = 0, e1 = KNOWN_COMMANDS - 1, c = -1, r = 0;
    GSCmd* cmd = NULL;

    // printf("CMD %c%c%c%c%c%c%c%c\n",gs.buffer[0],gs.buffer[1],gs.buffer[2],gs.buffer[3],gs.buffer[4],gs.buffer[5],gs.buffer[6],gs.buffer[7]);
    while (e0 <= e1) {
        c = (e0 + e1) / 2;
        cmd = &gs_commands[c];
        //for this to work the first 16 bytes of gs.buffer must be zeroed at each read!
        //otherwise previous bytes can interfere
        r = memcmp(gs.buffer, cmd->body, cmd->len);
        if (r == 0 && gs.buffer[cmd->len] != ':') {
            //ouch! it means cmd is a prefix of a longer command -_-
            //so command is less than gs.buffer
            //therefore set r to +1
            printf("OUCH!\n");
            r = 1;
        }
        if (r > 0)
            e0 = c + 1;
        else if (r < 0)
            e1 = c - 1;
        else
            break;
    }
    if (e0 <= e1) {
        printf("RET CMD %i\n", c);
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
uint8_t* _gs_advance_to(uint8_t* buf, uint8_t* ebuf, uint8_t* pattern)
{
    uint8_t* pt;
    while (buf < ebuf) {
        pt = pattern;
        while (*pt) {
            if (*buf == *pt)
                return buf;
            pt++;
        }
        buf++;
    }
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
uint8_t* _gs_findstr(uint8_t* buf, uint8_t* ebuf, uint8_t* pattern)
{
    uint8_t* pt;
    while (buf < ebuf) {
        pt = pattern;
        while (*pt && buf < ebuf) {
            if (*buf != *pt)
                break;
            pt++;
            buf++;
        }
        if (!*pt)
            return buf;
        buf++;
    }
    return NULL;
}

/**
 * @brief parse numbers in base 10 from a byte buffe 
 *
 * Does not check for number format correctness (0003 is ok) and
 * does also parse negative numbers. Whitespace chars ' ','\r','\n'
 * break the number, other chars before end point return NULL.
 *
 * @param[in]  buf     starting point
 * @param[in]  ebuf    ending point (not included)
 * @param[out] result  the positon reached after the parsing, NULL on failed parsing
 *
 * @return 
 */
uint8_t* _gs_parse_number(uint8_t* buf, uint8_t* ebuf, int32_t* result)
{
    int res = 0;
    int found = 0;
    int sign=1;
    while (buf < ebuf) {
        if (*buf >= '0' && *buf <= '9') {
            found = 1;
            res = res * 10 + (*buf - '0');
        } else if(*buf=='-'){
            if (found)
            return NULL; //allow spaces
            found = 1;
            sign=-1;
        } else if (*buf == ' ') {
            if (found)
                break;
        } else if (*buf == '\r' || *buf == '\n') {
            if (found)
                break;
            else
                return NULL;
        } else {
            return NULL;
        }
        buf++;
    }
    if (result)
        *result = res*sign;
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
int _gs_parse_command_arguments(uint8_t* buf, uint8_t* ebuf, const char* fmt, ...)
{
    va_list vl;
    va_start(vl, fmt);
    int32_t ret = 0;
    int32_t* iparam;
    uint8_t** sparam;
    uint8_t* pms;
    uint8_t* pme = ebuf;
    int i;
    pms = buf;
    while (buf < ebuf) {
        buf = _gs_advance_to(buf, ebuf, ",\r\n");
        if (!buf)
            break;
        pme = buf - 1;
        // *buf=0;
        switch (*fmt) {
        case 0:
            goto exit;
        case 'i':
            iparam = va_arg(vl, int32_t*);
            if (iparam)
                *iparam = 0;
            pms = _gs_parse_number(pms, pme + 1, iparam);
            if (!pms) {
                goto exit;
            }
            ret++;
            break;
        case 'S': // remove quotes if present
            if (*pms == '\"')
                pms++;
            if (*pme == '\"')
                pme--;
            // fall down
        case 's':
            sparam = va_arg(vl, uint8_t**);
            iparam = va_arg(vl, int32_t*);
            if (sparam)
                *sparam = pms;
            // *buf=0; //end string
            if (iparam)
                *iparam = pme - pms + 1; //store len
            ret++;
            break;
        }
        fmt++;
        pms = ++buf;
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
 * @param[i] fmt     format string (can be NULL)
 * @param[i] ...     variadic arguments
 */
void _gs_send_at(int cmd_id, const char* fmt, ...)
{
    static uint8_t _strbuf[16];
    GSCmd* cmd = GS_GET_CMD(cmd_id);
    uint8_t* sparam;
    int32_t iparam;
    int32_t iparam_len;
    va_list vl;
    va_start(vl, fmt);

    vosSemWait(gs.sendlock);
    vhalSerialWrite(gs.serial, "AT", 2);
    printf("->: AT");
    vhalSerialWrite(gs.serial, cmd->body, cmd->len);
    printf("%s", cmd->body);
    if (fmt)
    while (*fmt) {
        switch (*fmt) {
        case 'i':
            //number
            iparam = va_arg(vl, int32_t);
            iparam_len = modp_itoa10(iparam, _strbuf);
            vhalSerialWrite(gs.serial, _strbuf, iparam_len);
            _strbuf[iparam_len] = 0;
            printf("%s", _strbuf);
            break;
        case 's':
            sparam = va_arg(vl, uint8_t*);
            iparam_len = va_arg(vl, int32_t);
            vhalSerialWrite(gs.serial, sparam, iparam_len);
            print_buffer(sparam, iparam_len);
            break;
        default:
            vhalSerialWrite(gs.serial, fmt, 1);
            printf("%c", *fmt);
        }
        fmt++;
    }
    vhalSerialWrite(gs.serial, "\r", 1);
    printf("\n");
    vosSemSignal(gs.sendlock);
    va_end(vl);
}

int _gs_wait_for_pin_ready(void)
{
    int i;
    for (i = 3; i > 0; --i) {
        _gs_empty_rx();
        vhalSerialWrite(gs.serial, "AT+CPIN?\r\n", 10);
        uint32_t tstart = vosMillis();
        do {
            if (_gs_readline(1000) >= 0) {
                if (_gs_findstr(gs.buffer, gs.buffer + gs.bytes, "+CPIN: READY")) {
                    if (!_gs_wait_for_ok(500))
                        continue;
                    i = -1; // success, exit!
                    break;
                }
            }
        } while ((vosMillis() - tstart) < 5000); // max reply time 5s
    }
    _gs_empty_rx();
    return (i < 0);
}

int _gs_get_initialization_status(void)
{
    uint8_t* p;
    int32_t sta = -1; // unexpected/unknown
    int i;

    vhalSerialWrite(gs.serial, "AT+QINISTAT\r\n", 13);
    for (i = 10; i > 0; --i) {
        if (_gs_readline(100) >= 0) {
            p = _gs_findstr(gs.buffer, gs.buffer + gs.bytes, "+QINISTAT:");
            if (p && _gs_parse_number(p, gs.buffer + gs.bytes, &sta))
                break;
        }
    }
    if (!_gs_wait_for_ok(500))
        return -1;
    return sta;
}

/**
 * @brief Configure basic parameters for startup
 *
 * Disables echo, set CMEE to 2, set urcs and displays info about firmware
 *
 * @return 0 on failure
 */
int _gs_config0(int min_fun)
{
    //clean serial
    int i;
    //autobaud (max 10 seconds)
    for (i = 0; i < 50; i++) {
        vhalSerialWrite(gs.serial, "ATE1\r\n", 6);
        printf(".\n");
        if (_gs_readline(200) >= 0) {
            if (_gs_findstr(gs.buffer, gs.buffer + gs.bytes, "ATE1")) {
                if (_gs_wait_for_ok(200))
                    break;
            }
        }
    }
    // discard any rubbish
    vosThSleep(TIME_U(500, MILLIS));
    _gs_empty_rx();

    //disable echo
    vhalSerialWrite(gs.serial, "ATE0\r\n", 6);
    if (!_gs_wait_for_ok(500))
        return 0;

    //fix baud rate, just to be sure
    vhalSerialWrite(gs.serial, "AT+IPR=115200\r\n", 15);
    if (!_gs_wait_for_ok(500))
        return 0;

    if (min_fun) {
        //enter minimal functionality
        _gs_send_at(GS_CMD_CFUN, "=0");
        if (!_gs_wait_for_ok(5000))
            return 0;
    }
    else {
        //enter full functionality
        _gs_send_at(GS_CMD_CFUN, "=1");
        if (!_gs_wait_for_ok(15500))
            return 0;
    }

    //full error messages
    _gs_send_at(GS_CMD_CMEE, "=i", 2);
    if (!_gs_wait_for_ok(500))
        return 0;

    //enable urc about network status
    _gs_send_at(GS_CMD_CREG, "=i", 2);
    if (!_gs_wait_for_ok(500))
        return 0;
    _gs_send_at(GS_CMD_CGREG, "=i", 2);
    if (!_gs_wait_for_ok(500))
        return 0;
    _gs_send_at(GS_CMD_CEREG, "=i", 2);
    if (!_gs_wait_for_ok(500))
        return 0;

    //display product ID
    vhalSerialWrite(gs.serial, "ATI\r\n", 5);
    if (!_gs_wait_for_ok(500))
        return 0;
    vhalSerialWrite(gs.serial, "AT+QGMR\r\n", 9);
    _gs_wait_for_ok(500);
    
    // minimal functionality configuration ends here
    if (min_fun)
        return 1;

    // wait for PIN ready
    if (!_gs_wait_for_pin_ready())
        return 0;
    // wait initialization complete
   for (i = 50; i > 0; --i) {
        int sta = _gs_get_initialization_status();
        if (sta >= 3)
            break;
        vosThSleep(TIME_U(100, MILLIS));
    }
    if (i == 0)
        return 0;

    //timezone update
    vhalSerialWrite(gs.serial, "AT+CTZU=1\r\n", 11);
    if (!_gs_wait_for_ok(1500))
        return 0;

    //set sms format
    vhalSerialWrite(gs.serial, "AT+CMGF=1\r\n", 11);
    if (!_gs_wait_for_ok(500))
        return 0;

    //set text encoding
    vhalSerialWrite(gs.serial, "AT+CSCS=\"IRA\"\r\n", 15);
    if (!_gs_wait_for_ok(500))
        return 0;

    //get scsa
    vhalSerialWrite(gs.serial, "AT+CSCA?\r\n", 10);
    if (!_gs_wait_for_ok(500))
        return 0;

    //setup sms
    vhalSerialWrite(gs.serial, "AT+CNMI=2,1,0,0,0\r\n", 19);
    if (!_gs_wait_for_ok(500))
        return 0;

    //enable urc about pdp status
    _gs_send_at(GS_CMD_CGEREP, "=i", 2);
    if (!_gs_wait_for_ok(500))
        return 0;

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
int _gs_valid_command_response(GSCmd* cmd)
{
    if (gs.buffer[cmd->len] == ':' && gs.buffer[cmd->len + 1] == ' ' && gs.bytes >= cmd->len + 2) {
        //valid
        return cmd->len + 2;
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
void _gs_handle_urc(GSCmd* cmd)
{
    int32_t p0, p1, p2, p3, nargs;
    uint8_t *s0, *s1, *s2, *s3;
    int p = _gs_valid_command_response(cmd);
    uint8_t* buf = gs.buffer + p;
    uint8_t* ebuf = gs.buffer + gs.bytes;
    GSocket* sock;

    if (!p)
        return;

    switch (cmd->id) {
    case GS_CMD_CMTI:
        //incoming sms
        gs.pendingsms++;
        break;

    case GS_CMD_QIOPEN:
    case GS_CMD_QSSLOPEN:
        printf("GS_CMD_QSSLOPEN\n");
        nargs = _gs_parse_command_arguments(buf, ebuf, "ii", &p0, &p1);
        if (nargs != 2)
            goto exit_err;
        if (p1 == 0) {
            //socket opened!
            _gs_socket_opened(p0, 1);
        } else {
            //socket error!
            _gs_socket_opened(p0, 0);
        }
        break;
    case GS_CMD_QIURC:
    case GS_CMD_QSSLURC:
        nargs = _gs_parse_command_arguments(buf, ebuf, "s", &s0, &p0);
        if (nargs != 1)
            goto exit_err;
        if (p0 == 8 && memcmp(s0, "\"closed\"", p0) == 0) {
            //socket closed!
            _gs_parse_command_arguments(buf, ebuf, "si", &s0, &p0, &p1);
            _gs_socket_closing(p1);
        } else if (p0 == 6 && memcmp(s0, "\"recv\"", p0) == 0) {
            //data ready!
            _gs_parse_command_arguments(buf, ebuf, "si", &s0, &p0, &p1);
            _gs_socket_pending(p1);
        } else if (p0 == 8 && memcmp(s0, "\"dnsgip\"", p0) == 0) {
            //dns ready!
            _gs_parse_command_arguments(buf, ebuf, "ss", &s0, &p0, &s1, &p1);
            if (s1[0] == '0') {
                //ok...get ipcount
                _gs_parse_command_arguments(buf, ebuf, "ssi", &s0, &p0, &s1, &p1, &p2);
                printf("Set dns count %i\n", p2);
                gs.dns_count = p2;
            } else {
                gs.dns_count--;
                if (s1[0] == '"') {
                    //it's an IP
                    printf("DNS %x -> %x of %i\n", s1, gs.dnsaddr, p1 - 2);
                    memcpy(gs.dnsaddr, s1 + 1, p1 - 2); //rermove quotes around ip
                    gs.dnsaddrlen = p1 - 2;
                } else {
                    // errors or no idea
                    gs.dnsaddrlen = 0;
                    gs.dns_count = 0; //exit
                }
                printf("DNS COUNT %i\n", gs.dns_count);
                if (!gs.dns_count)
                    gs.dns_ready = 1;
            }
        } else if (p0 == 10 && memcmp(s0, "\"pdpdeact\"", p0) == 0) {
            //pdp deactivated, set all sock closing
            printf("PDP DEACTIVATED\n");
            _gs_socket_close_all();
        } else {
            //socket error!
            _gs_socket_opened(p0, 0);
        }
        break;
    case GS_CMD_CREG:
        //retrieve status of network registration
        _gs_set_gsm_status_from_creg(buf, ebuf, 1);
        break;
    case GS_CMD_CGREG:
        //retrieve status of gprs network registration
        _gs_set_gprs_status_from_cgreg(buf, ebuf, 1);
        break;
    case GS_CMD_CEREG:
        //retrieve status of gprs network registration
        _gs_set_eps_status_from_cereg(buf, ebuf, 1);
        break;
    case GS_CMD_CGEV:
        //retrieve status of pdp activation
        if (memcmp(buf + 3, "DETACH", 6) == 0) {
            //pdp detach
            printf("PDP DETACH\n");
            _gs_socket_close_all();
        } else if (memcmp(buf + 3, "DEACT", 5) == 0) {
            //pdp deact
            printf("PDP DEACT\n");
            _gs_socket_close_all();
        } else if (memcmp(buf + 3, "CLASS", 5) == 0) {
            //change of class
            printf("PDP CLASS\n");
        }
        break;

    default:
        printf("Unhandled URC %i\n", cmd->id);
    }

exit_ok:
    return;

exit_err:
    printf("Error parsing arguments for %i\n", cmd->id);
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
uint8_t _slotbuf[MAX_CMD];
GSSlot* _gs_acquire_slot(int cmd_id, uint8_t* respbuf, int max_size, int timeout, int nparams)
{
    vosSemWait(gs.slotlock);
    gslot.cmd = GS_GET_CMD(cmd_id);
    gslot.stime = vosMillis();
    gslot.timeout = timeout;
    gslot.has_params = nparams;
    if (!respbuf) {
        if (max_size) {
            gslot.resp = _slotbuf; //gc_malloc(max_size);
            gslot.eresp = gslot.resp;
        } else {
            gslot.resp = gslot.eresp = NULL;
        }
        gslot.allocated = 1;
        gslot.max_size = max_size;
    } else {
        gslot.resp = gslot.eresp = respbuf;
        gslot.max_size = max_size;
        gslot.allocated = 0;
    }

    gs.slot = &gslot;
    return &gslot;
}

/**
 * @brief Wait until the main thread signal of slot completion
 */
void _gs_wait_for_slot(void)
{
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
int _gs_wait_for_slot_mode(uint8_t* text, int32_t textlen, uint8_t* addtxt, int addtxtlen)
{
    //can be polled!
    int cnt = 0;
    printf("Waiting for mode\n");

    // vhalSerialWrite(gs.serial,">",1);
    while (gs.mode == GS_MODE_NORMAL && cnt < 100) { //after 10 seconds, timeout
        vosThSleep(TIME_U(100, MILLIS));
        cnt++;
    }

    if (gs.mode != GS_MODE_PROMPT)
        return -1;
    printf("Slot wait mode\n-->");
    // print_buffer(text,textlen);
    printf("\n");

    while (textlen > 0) {
        cnt = MIN(64, textlen);
        printf("Sending %i\n", cnt);
        cnt = vhalSerialWrite(gs.serial, text, cnt);
        printf("Sent %i\n", cnt);
        textlen -= cnt;
        text += cnt;
        printf("Remaining %i\n", textlen);
    }
    while (addtxtlen > 0) {
        cnt = MIN(64, addtxtlen);
        printf("Sending %i\n", cnt);
        cnt = vhalSerialWrite(gs.serial, addtxt, cnt);
        printf("Sent %i\n", cnt);
        addtxtlen -= cnt;
        addtxt += cnt;
        printf("Remaining %i\n", addtxtlen);
    }
    gs.mode = GS_MODE_NORMAL; //back to normal mode

    return 0;
}

int _gs_wait_for_buffer_mode(void)
{
    //can be polled!
    int cnt = 0;
    printf("Waiting for buffer mode\n");
    while (gs.mode != GS_MODE_BUFFER && cnt < 100) { //after 10 seconds, timeout
        vosThSleep(TIME_U(100, MILLIS));
        cnt++;
    }

    return gs.mode == GS_MODE_BUFFER;
}

int _gs_write_in_buffer_mode(uint8_t* buf, int len)
{
    if (buf && len) {
        vhalSerialWrite(gs.serial, buf, len);
    }
    return 0;
}

int _gs_exit_from_buffer_mode_w(uint8_t* buf, int len)
{

    if (buf && len) {
        vhalSerialWrite(gs.serial, buf, len);
    }
    gs.mode = GS_MODE_NORMAL;
    vosSemSignal(gs.bufmode);
    return 0;
}

int _gs_exit_from_buffer_mode_r(uint8_t* buf, int len, int max, GSocket* sock)
{

    if (max < len)
        len = max;
    if (buf && len) {
        printf("bmode read %i\n", len);
        int rd = vhalSerialRead(gs.serial, buf, len);
        printf("sock %x %i %i %i\n", sock, max, len, rd);
        if (sock) {
            int pos=-1;
            while (max > len) {
                pos = (sock->head + sock->len) % MAX_SOCK_RX_BUF;
                printf("bmode read 1 at %i/%i pos %i\n", sock->head, sock->len, pos);
                vhalSerialRead(gs.serial, sock->rxbuf + pos, 1);
                sock->len++;
                max--;
            }
            //signal that there is pending data in the buffer
            if(pos>=0) vosSemSignal(gs.selectlock);
        } else {
            while (max > len) {
                //skip up to max
                uint8_t dummy;
                vhalSerialRead(gs.serial, &dummy, 1);
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
void _gs_release_slot(GSSlot* slot)
{
    // if (slot->allocated && slot->resp) gc_free(slot->resp);
    memset(slot, 0, sizeof(GSSlot));
    vosSemSignal(gs.slotlock);
}

/**
 * @brief Signal the current slot as ok
 */
void _gs_slot_ok(void)
{
    printf("ok slot %s\n", gs.slot->cmd->body);
    gs.slot->err = 0;
    gs.slot = NULL;
    vosSemSignal(gs.slotdone);
}

/**
 * @brief Signal the current slot as error
 */
void _gs_slot_error(void)
{
    printf("error slot %s\n", gs.slot->cmd->body);
    gs.slot->err = GS_ERR_INVALID;
    gs.slot = NULL;
    vosSemSignal(gs.slotdone);
}

/**
 * @brief Signal the current slot as timed out
 */
void _gs_slot_timeout(void)
{
    printf("timeout slot %s\n", gs.slot->cmd->body);
    gs.slot->err = GS_ERR_TIMEOUT;
    gs.slot = NULL;
    vosSemSignal(gs.slotdone);
}

/**
 * @brief Transfer the command response in gs.buffer to the slot memory
 *
 * @param[in] cmd the command to transfer
 */
void _gs_slot_params(GSCmd* cmd)
{
    //copy params to slot
    if (!gs.slot->resp) {
        return;
    }
    if (cmd->response_type == GS_RES_STR || cmd->response_type == GS_RES_STR_OK) {
        if (memcmp(gs.buffer, "+QIND", 5) == 0) {
            // skip invalid param
            printf("unknown URC: %s\n", gs.buffer);
            return;
        }
        int csize = (gs.slot->max_size < gs.bytes) ? gs.slot->max_size : gs.bytes;
        memcpy(gs.slot->resp, gs.buffer, csize);
        gs.slot->eresp = gs.slot->resp + csize;
        // printf("Copy command %s %i %c%c%c\n",cmd->body,csize,*gs.slot->resp,*(gs.slot->resp+1),*(gs.slot->resp+2));
    } else {
        if (!_gs_valid_command_response(cmd))
            return;
        int psize = gs.bytes - cmd->len - 2;
        int csize = (gs.slot->max_size < psize) ? gs.slot->max_size : psize;
        memcpy(gs.slot->resp, gs.buffer + cmd->len + 2, csize);
        gs.slot->eresp = gs.slot->resp + csize;
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
void _gs_loop(void* args)
{
    (void)args;
    GSCmd* cmd;
    printf("_gs_loop started (Thread %d)\n", vosThGetId(vosThCurrent()));
    while (gs.initialized) {
        // do nothing if serial is not active
        if (!gs.talking) {
            gs.running = 0;
            vosThSleep(TIME_U(500, MILLIS));
            continue;
        }
        gs.running = 1;
        // printf("looping\n");
        if (gs.mode == GS_MODE_NORMAL) {
            if (_gs_readline(100) <= 3) {
                if (
                    gs.bytes >= 1 && gs.buffer[0] == '>' && gs.slot && (gs.slot->cmd->id == GS_CMD_QISEND || gs.slot->cmd->id == GS_CMD_QSSLSEND || gs.slot->cmd->id == GS_CMD_CMGS)) {
                    //only enter in prompt mode if the current slot is for QISEND/CMGS to avoid locks
                    printf("GOT PROMPT!\n");
                    gs.mode = GS_MODE_PROMPT;
                    continue;
                }
                //no line
                if (gs.slot) {
                    if (gs.slot->timeout && (vosMillis() - gs.slot->stime) > gs.slot->timeout) {
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
                    //we parsed a command
                    if (cmd == gs.slot->cmd) {
                        //we parsed the response to slot
                        if (gs.slot->has_params) {
                            printf("filling slot params for %s\n", cmd->body);
                            _gs_slot_params(cmd);
                            if (cmd->id == GS_CMD_QIRD || cmd->id == GS_CMD_QSSLRECV) {
                                //it's a QIRD response, enter read buffer mode!
                                //unless it's just a check
                                gs.mode = GS_MODE_BUFFER;
                            } else if (cmd->id == GS_CMD_CMGL) {
                                int idx;
                                uint8_t *sta, *oa, *alpha, *scts;
                                int stalen, oalen, alphalen, sctslen;

                                printf("CMGL\n");
                                //we are reading sms list
                                if (_gs_parse_command_arguments(gs.slot->resp, gs.slot->eresp, "issss", &idx, &sta, &stalen, &oa, &oalen, &alpha, &alphalen, &scts, &sctslen) == 5) {
                                    printf("CMGL parsed\n");
                                    if (memcmp(sta + stalen - 5, "READ", 4) != 0) {
                                        //it's not a read or unread sms
                                        gs.skipsms = 1;
                                        printf("CMGL skip 1\n");

                                    } else {
                                        if (gs.cursms >= gs.maxsms - 1 || idx < gs.offsetsms) {
                                            gs.skipsms = 1;
                                            printf("CMGL skip 2\n");
                                        } else {
                                            printf("CMGL read\n");
                                            gs.skipsms = 0;
                                            gs.cursms++;
                                            //got a new sms
                                            //copy address
                                            memcpy(gs.sms[gs.cursms].oaddr, oa + 1, MIN(oalen - 2, 16));
                                            gs.sms[gs.cursms].oaddrlen = MIN(oalen - 2, 16);
                                            //copy time
                                            memcpy(gs.sms[gs.cursms].ts, scts + 1, MIN(sctslen - 2, 24));
                                            gs.sms[gs.cursms].tslen = MIN(sctslen - 2, 24);

                                            gs.sms[gs.cursms].index = idx;

                                            if (sta[5] == 'U') {
                                                gs.sms[gs.cursms].unread = 1;
                                            } else {
                                                gs.sms[gs.cursms].unread = 0;
                                            }
                                        }
                                    }
                                }
                            }
                        } else {
                            printf("Unexpected params for slot\n");
                        }
                    } else if (cmd->urc & GS_CMD_URC) {
                        //we parsed a urc
                        printf("Handling urc %s in a slot\n", cmd->body);
                        _gs_handle_urc(cmd);
                    }
                } else {
                    //we don't have a command
                    if (_gs_check_ok()) {
                        //we got an OK...is it for the current slot?
                        if (gs.slot->has_params == gs.slot->params) {
                            _gs_slot_ok();
                        } else {
                            if (gs.slot->cmd->id == GS_CMD_CMGL || gs.slot->cmd->id == GS_CMD_QENG) {
                                //variable args
                                _gs_slot_ok();
                            } else {
                                printf("Unexpected OK %s %i %i\n", gs.slot->cmd->body, gs.slot->params, gs.slot->has_params);
                            }
                        }
                    } else if (_gs_check_error()) {
                        _gs_slot_error();
                    } else if (gs.slot->cmd->response_type == GS_RES_STR) {
                        //the command behaves differently
                        printf("filling slot params for GS_RES_STR\n");
                        _gs_slot_params(gs.slot->cmd);
                        _gs_slot_ok();
                    } else if (gs.slot->cmd->response_type == GS_RES_STR_OK) {
                        //the command behaves differently
                        printf("filling slot params for GS_RES_STR_OK\n");
                        _gs_slot_params(gs.slot->cmd);
                    } else {
                        if (gs.slot->cmd->id == GS_CMD_QFUPL && memcmp(gs.buffer, "CONNECT", 7) == 0) {
                            // go in buffer mode
                            gs.mode = GS_MODE_BUFFER;
                        } else if (gs.slot->cmd->id == GS_CMD_CMGL) {
                            //it's a line of text, read the sms
                            if (gs.skipsms) {
                                printf("Skip sms\n");
                            } else {
                                printf("reading sms %i\n", gs.bytes);
                                memcpy(gs.sms[gs.cursms].txt, gs.buffer, MIN(gs.bytes - 2, 160));
                                gs.sms[gs.cursms].txtlen = MIN(gs.bytes - 2, 160);
                            }
                        } else {
                            printf("Unexpected line\n");
                        }
                    }
                }
            } else {
                // we have no slot
                if (cmd) {
                    //we have a command
                    if (cmd->urc & GS_CMD_URC) {
                        printf("Handling urc %s out of slot\n", cmd->body);
                        _gs_handle_urc(cmd);
                    } else {
                        printf("Don't know what to do with %s\n", cmd->body);
                    }
                } else {
                    // we have no command
                    printf("Unknown line out of slot\n");
                }
            }
        } else if (gs.mode == GS_MODE_PROMPT) {
            //// PROMPT MODE
            //Prompt mode is used for USECMNG (implemented) and DWNFILE (not implemented)
            //If needed, logic for prompt mode goes here
            int ss;
            for (ss = 0; ss < 40; ss++) {
                //avoid locking, max time spendable in prompt mode = 20s
                vosThSleep(TIME_U(500, MILLIS));
                if (gs.mode != GS_MODE_PROMPT)
                    break;
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
int _gs_socket_new(int proto, int secure)
{
    GSocket* sock;
    int res = -1;
    int i = 0;
    //Refuse a new socket when network is down
    if (IS_NETWORK_UNREGISTERED_SINCE_TOO_LONG()){
        printf("can't open socket, no network\n");
        return res;
    }
    //TODO: protect with global semaphore
    for (i = 0; i < MAX_SOCKS; i++) {
        sock = &gs_sockets[i];
        // vosSemWait(sock->lock);
        if (!sock->acquired) {

            if (sock->to_be_closed) {
                _gs_socket_close(i);
            }
            sock->acquired = 1;
            sock->to_be_closed = 0;
            sock->connected = 0;
            sock->timeout = 0;
            sock->bound = 0;
            sock->secure = secure;
            sock->proto = proto;
            sock->head = 0;
            sock->len = 0;
            res = i;
            // vosSemSignal(sock->lock);
            break;
        }
        // vosSemSignal(sock->lock);
    }

    return res;
}

int _gs_file_delete(uint8_t* filename, int namelen)
{
    GSSlot* slot;
    int res = 0;

    slot = _gs_acquire_slot(GS_CMD_QFDEL, NULL, 0, GS_TIMEOUT, 0);
    _gs_send_at(GS_CMD_QFDEL, "=\"s\"", filename, namelen);
    _gs_wait_for_slot();
    res = slot->err;
    _gs_release_slot(slot);

    return res;
}

int _gs_file_upload(uint8_t* filename, int namelen, uint8_t* content, int len)
{
    GSSlot* slot;
    int res = 0;
    int snt = 0;
    int tsnd = 0;

    slot = _gs_acquire_slot(GS_CMD_QFUPL, NULL, 64, GS_TIMEOUT * 60, 1);
    _gs_send_at(GS_CMD_QFUPL, "=\"s\",i,5,0", filename, namelen, len);
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

uint8_t f_cacert[12] = "cacert#.pem";
uint8_t f_clicert[12] = "clicrt#.pem";
uint8_t f_prvkey[12] = "prvkey#.pem";

int _gs_ssl_cfg(int op, int ctx, int val)
{
    GSSlot* slot;
    int res = 0;

    //WARNING: names of certificates are global!

    slot = _gs_acquire_slot(GS_CMD_QSSLCFG, NULL, 0, GS_TIMEOUT * 5, 0);
    switch (op) {
    case 0:
        _gs_send_at(GS_CMD_QSSLCFG, "=\"s\",i,i", "sslversion", 10, ctx, val); //select TLS 1.2 only
        break;
    case 1:
        _gs_send_at(GS_CMD_QSSLCFG, "=\"s\",i,0XFFFF", "ciphersuite", 11, ctx); //select all secure ciphersuites
        break;
    case 2:
        _gs_send_at(GS_CMD_QSSLCFG, "=\"s\",i,\"s\"", "cacert", 6, ctx, f_cacert, 7); //select cacert
        break;
    case 3:
        _gs_send_at(GS_CMD_QSSLCFG, "=\"s\",i,\"s\"", "clientcert", 10, ctx, f_clicert, 7); //select clicert
        break;
    case 4:
        _gs_send_at(GS_CMD_QSSLCFG, "=\"s\",i,\"s\"", "clientkey", 9, ctx, f_prvkey, 7); //select prvkey
        break;
    case 5:
        _gs_send_at(GS_CMD_QSSLCFG, "=\"s\",i,i", "seclevel", 8, ctx, val); //select prvkey
        break;
    case 6:
        _gs_send_at(GS_CMD_QSSLCFG, "=\"s\",i,i", "ignorelocaltime", 15, ctx, val); //select date check
        break;
    case 7:
        _gs_send_at(GS_CMD_QSSLCFG, "=\"s\",i,i", "negotiatetime", 13, ctx, val); //select timeout
        break;
    }
    _gs_wait_for_slot();
    res = slot->err;
    _gs_release_slot(slot);
    return res;
}

int _gs_socket_tls(int id, uint8_t* cacert, int cacertlen, uint8_t* clicert, int clicertlen, uint8_t* pvkey, int pvkeylen, int authmode)
{
    GSocket* sock;
    GSSlot* slot;
    int res = 0;
    int ctx = id;
    sock = &gs_sockets[id];

    //SSL CTX = socket id
    //let's upload files..

    vosSemWait(sock->lock);

    res += _gs_ssl_cfg(0, ctx, 3); //TLS 1.2
//    vosThSleep(TIME_U(1000, MILLIS));
    res += _gs_ssl_cfg(1, ctx, 0); //all ciphers

    if (cacert && cacertlen) {
        f_cacert[6] = '0' + id;
        _gs_file_delete(f_cacert, 7);
        res += _gs_file_upload(f_cacert, 7, cacert, cacertlen);
        res += _gs_ssl_cfg(2, ctx, 0);
    }
    if (clicert && clicertlen) {
        f_clicert[6] = '0' + id;
        _gs_file_delete(f_clicert, 7);
        res += _gs_file_upload(f_clicert, 7, clicert, clicertlen);
        res += _gs_ssl_cfg(3, ctx, 0);
    }
    if (pvkey && pvkeylen) {
        f_prvkey[6] = '0' + id;
        _gs_file_delete(f_prvkey, 7);
        res += _gs_file_upload(f_prvkey, 7, pvkey, pvkeylen);
        res += _gs_ssl_cfg(4, ctx, 0);
    }

    res += _gs_ssl_cfg(5, ctx, authmode); //0 none, 1 server, 2 server+client
    res += _gs_ssl_cfg(6, ctx, 1);        //ignore time check
    vosSemSignal(sock->lock);

    return res;
}

int _gs_socket_opened(int id, int success)
{
    printf("_gs_socket_opened %d %d\n", id, success);
    GSocket* sock;
    sock = &gs_sockets[id];
    if (success)
        sock->connected = 1;
    else
        sock->connected = 2;
    return 0;
}

int _gs_socket_bind(int id, struct sockaddr_in* addr)
{
    int res = 0;
    int timeout = 160000; //150s timeout for URC
    GSocket* sock;
    GSSlot* slot;
    sock = &gs_sockets[id];
    if (IS_NETWORK_UNREGISTERED_SINCE_TOO_LONG()){
        printf("can't bind socket, no network\n");
        return -1;
    }

    vosSemWait(sock->lock);

    slot = _gs_acquire_slot(GS_CMD_QIOPEN, NULL, 0, GS_TIMEOUT * 60 * 3, 0);
    if (sock->proto == 17) {
        //addr is ignored, we can only bind to 127.0.0.1
        _gs_send_at(GS_CMD_QIOPEN, "=i,i,\"UDP SERVICE\",\"127.0.0.1\",0,i,0", GS_PROFILE, id, OAL_GET_NETPORT(addr->sin_port));
    }
    _gs_wait_for_slot();
    if (slot->err) {
        res = -1;
    }
    _gs_release_slot(slot);

    vosSemSignal(sock->lock);
    if (res == -1) {
        return res;
    }

    res = -1;

    while (timeout > 0) {
        vosThSleep(TIME_U(100, MILLIS));
        timeout -= 100;
        vosSemWait(sock->lock);
        if (sock->connected) {
            if (sock->connected == 2) {
                res = -2;
            } else {
                res = 0;
            }
        }
        vosSemSignal(sock->lock);
        if (!res || res == -2)
            break;
    }

    if (res) {
        //oops, timeout or error
        vosSemWait(sock->lock);
        // close to allow retrying a new connect on the same sock
        _gs_do_close(id);
        sock->bound = 0;
        vosSemSignal(sock->lock);
    } else {
        sock->bound = 1;
    }
    printf("return from bind");
    return res;
}

int _gs_socket_connect(int id, struct sockaddr_in *addr){
    uint8_t saddr[16];
    uint32_t saddrlen;
    int res = 0;
    int timeout = 160000; //150s timeout for URC
    GSocket* sock;
    GSSlot* slot;

    if (IS_NETWORK_UNREGISTERED_SINCE_TOO_LONG()){
        printf("can't connect socket, no network\n");
        return -1;
    }

    saddrlen = zs_addr_to_string(addr, saddr);
    sock = &gs_sockets[id];
    vosSemWait(sock->lock);
    if (sock->secure) {
        slot = _gs_acquire_slot(GS_CMD_QSSLOPEN, NULL, 0, GS_TIMEOUT * 60 * 3, 0);
        if (sock->proto == 6) {
            _gs_send_at(GS_CMD_QSSLOPEN, "=i,i,i,\"s\",i", GS_PROFILE, id, id, saddr, saddrlen, OAL_GET_NETPORT(addr->sin_port));
        }
        //NO DTLS!!
        // else {
        //     _gs_send_at(GS_CMD_QIOPEN,"=i,i,i,\"UDP\",\"s\",i",GS_PROFILE,id,saddr,saddrlen,OAL_GET_NETPORT(addr->port));
        // }
    } else {
        slot = _gs_acquire_slot(GS_CMD_QIOPEN, NULL, 0, GS_TIMEOUT * 60 * 3, 0);
        if (sock->proto == 6) {
            _gs_send_at(GS_CMD_QIOPEN, "=i,i,\"TCP\",\"s\",i,0,0", GS_PROFILE, id, saddr, saddrlen, OAL_GET_NETPORT(addr->sin_port));
        } else {
            //udp
            _gs_send_at(GS_CMD_QIOPEN, "=i,i,\"UDP\",\"s\",i,0,0", GS_PROFILE, id, saddr, saddrlen, OAL_GET_NETPORT(addr->sin_port));
        }
    }
    _gs_wait_for_slot();
    if (slot->err) {
        res = -1;
    }
    _gs_release_slot(slot);

    vosSemSignal(sock->lock);
    if (res == -1) {
        return res;
    }

    res = -1;

    while (timeout > 0) {
        vosThSleep(TIME_U(100, MILLIS));
        timeout -= 100;
        vosSemWait(sock->lock);
        if (sock->connected) {
            if (sock->connected == 2) {
                res = -2;
            } else {
                res = 0;
            }
        }
        vosSemSignal(sock->lock);
        if (!res || res == -2)
            break;
    }

    if (res) {
        _gs_socket_close(id);
        // vosSemWait(sock->lock);
        // if connection failed, close to allow retrying a new connect on the same sock
        // _gs_do_close(id);
        // sock->connected = 0;
        // vosSemSignal(sock->lock);
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
GSocket* _gs_socket_get(int id)
{
    GSocket *sock, *res = NULL;

    sock = &gs_sockets[id];
    vosSemWait(sock->lock);

    if (sock->acquired)
        res = sock;

    vosSemSignal(sock->lock);
    return res;
}

/**
 * @brief Send command to close the socket (use internally)
 *
 * @param id index of the socket
 */
int _gs_do_close(int id)
{
    GSocket* sock;
    GSSlot* slot;
    int res = 0;
    sock = &gs_sockets[id];

    if (sock->secure) {
        slot = _gs_acquire_slot(GS_CMD_QSSLCLOSE, NULL, 0, GS_TIMEOUT * 15, 0);
        _gs_send_at(GS_CMD_QSSLCLOSE, "=i,10", id);
    } else {
        slot = _gs_acquire_slot(GS_CMD_QICLOSE, NULL, 0, GS_TIMEOUT * 15, 0);
        _gs_send_at(GS_CMD_QICLOSE, "=i,10", id);
    }
    _gs_wait_for_slot();
    if (slot->err) {
        res = -1;
    }
    _gs_release_slot(slot);

    if (res == 0) {
        sock->connected = 0;
        sock->bound = 0;
    }

    return res;
}

int _gs_socket_close_nolock(int id){
    GSocket* sock = &gs_sockets[id];
    //if not acquired, ignore
    if (!sock->acquired) return 0;
    int res = _gs_do_close(id);
    //regardless of the error (already closed), release this socket index
    sock->acquired = 0;
    //unlock sockets waiting on rx
    vosSemSignal(sock->rx);
    return res;
}

/**
 * @brief Close socket and release resources
 *
 * Even after a socket connection has been closed remotely, call this function
 * to free the allocated socket resource.
 *
 * @param id index of the socket
 */
int _gs_socket_close(int id)
{
    GSocket* sock;
    // GSSlot *slot;
    int res = 0;
    sock = &gs_sockets[id];

    vosSemWait(sock->lock);
    /*res =*/ _gs_socket_close_nolock(id);
    vosSemSignal(sock->lock);
    return res;
}

void _gs_socket_close_all(void)
{
    GSocket* sock;
    // GSSlot *slot;
    int id;

    printf("Closing all sockets...\n");
    for (id = 0; id < MAX_SOCKS; id++) {
        sock = &gs_sockets[id];
        if (sock->acquired) {
            _gs_socket_closing(id);
        }
    }
}

int _gs_socket_send(int id, uint8_t* buf, int len)
{
    int res = len;
    GSSlot* slot;
    GSocket* sock;
    sock = &gs_sockets[id];

    vosSemWait(sock->lock);
    CHECK_SOCKET_OPEN(sock);
    if (sock->to_be_closed) {
        // _gs_socket_close_nolock(id);
        res = -1;
    } else {
        if (sock->secure) {
            slot = _gs_acquire_slot(GS_CMD_QSSLSEND, NULL, 32, GS_TIMEOUT * 10, 1);
            _gs_send_at(GS_CMD_QSSLSEND, "=i,i", id, len);
        } else {
            slot = _gs_acquire_slot(GS_CMD_QISEND, NULL, 32, GS_TIMEOUT * 10, 1);
            _gs_send_at(GS_CMD_QISEND, "=i,i", id, len);
        }
        res = _gs_wait_for_slot_mode(buf, len, NULL, 0);
        if (res) {
            //ouch!
            printf("OUCH %i\n", res);
        } else {
            res = len;
        }
        _gs_wait_for_slot();
        if (slot->err) {
            res = -1;
            _gs_socket_closing(id);
        } else {
            //check resp
            if (memcmp(slot->resp, "SEND FAIL", 9) == 0) {
                //buffer full!
                res = 0;
            }
            //also check network status
            if (IS_NETWORK_UNREGISTERED_SINCE_TOO_LONG()){
                printf("closing socket forcibly from send\n");
                _gs_socket_closing(id);
            }
        }
        _gs_release_slot(slot);
    }
    vosSemSignal(sock->lock);

    return res;
}

int _gs_socket_isalive(int id)
{
    int res = 1;
    GSSlot* slot;
    GSocket* sock;
    sock = &gs_sockets[id];

    vosSemWait(sock->lock);
    if (sock->to_be_closed) {
        //needs to be closed, set alive
        res = 1;
    } else {
        uint32_t tot;
        uint32_t ack;
        uint32_t unack;
        int cmdlen;
        if (sock->secure) {
            //QSSLSEND does not support keepalive -_- (and QISEND does not apparently work with ssl)
            //so we must return true
            res = 1;
        } else {
            slot = _gs_acquire_slot(GS_CMD_QISEND, NULL, 32, GS_TIMEOUT * 10, 1);
            _gs_send_at(GS_CMD_QISEND, "=i,0", id);
            cmdlen = 7;
            _gs_wait_for_slot();
            if (!slot->err) {
                if (memcmp(slot->resp, slot->cmd, cmdlen) == 0) {
                    //resp starts with +QISEND
                    //we need to check this because +QISEND=id,0 answers in a different way
                    //than +QISEND=id,len, so the loop cannot parse args correctly (GS_RES_STR vs GS_RES_STR_OK)
                    slot->resp += 8; //increment resp until after the colon
                    if (_gs_parse_command_arguments(slot->resp, slot->eresp, "iii", &tot, &ack, &unack) != 3) {
                        //error in command, assume it is not alive
                        res = 0;
                    } else {
                        //check unacks
                        printf("isalive %i %i %i %i\n", id, tot, ack, unack);
                        if (unack > MAX_UNACKED_DATA) {
                            //TODO: this threshold should be settable from python one day, ideally through setsockopt
                            //but with some gotchas: it is not a real keepalive unless one end of the socket
                            //is always reading and the other one is sending periodically...
                            //To implement real keepalive, we need the modem to send empty packets, but this is apparently not possible :(
                            res = 0;
                        } else {
                            res = 1;
                        }
                    }
                } else {
                    //bad response...ignore and set alive
                    res = 1;
                }
            } else {
                //generic error...assume it is still alive
                res = 1;
            }
            _gs_release_slot(slot);
        }
    }
    vosSemSignal(sock->lock);

    return res;
}

int _gs_socket_sendto(int id, uint8_t* buf, int len, struct sockaddr_in *addr)
{
    int res = len;
    int saddrlen;
    GSSlot* slot;
    GSocket* sock;
    uint8_t remote_ip[16];
    sock = &gs_sockets[id];

    saddrlen = zs_addr_to_string(addr, remote_ip);

    vosSemWait(sock->lock);
    CHECK_SOCKET_OPEN(sock);

    if (sock->to_be_closed) {
        // _gs_socket_close_nolock(id);
        res = -1;
    } else {
        slot = _gs_acquire_slot(GS_CMD_QISEND, NULL, 32, GS_TIMEOUT * 10, 1);
        _gs_send_at(GS_CMD_QISEND, "=i,i,\"s\",i", id, len, remote_ip, saddrlen, OAL_GET_NETPORT(addr->sin_port));
        res = _gs_wait_for_slot_mode(buf, len, NULL, 0);
        if (res) {
            //ouch!
        } else {
            res = len;
        }
        _gs_wait_for_slot();
        if (slot->err) {
            res = -1;
        } else {
            //check resp
            if (memcmp(slot->resp, "SEND FAIL", 9) == 0) {
                //buffer full!
                res = 0;
            }
            //also check network status
            if (IS_NETWORK_UNREGISTERED_SINCE_TOO_LONG()){
                printf("closing socket forcibly from sendto\n");
                _gs_socket_closing(id);
            }
        }
        _gs_release_slot(slot);
    }
    vosSemSignal(sock->lock);

    return res;
}

int _gs_socket_recvfrom(int id, uint8_t* buf, int len, struct sockaddr_in *addr){
    int res = len;
    int rd;
    int nargs;
    uint8_t* remote_ip;
    int remote_len;
    int remote_port;
    GSSlot* slot;
    GSocket* sock;

    sock = &gs_sockets[id];
    vosSemWait(sock->lock);
    CHECK_SOCKET_OPEN(sock);

    rd = _gs_sock_copy(id, buf, len);
    if (rd > 0) {
        //skip command
        res = rd;
    } else if (sock->to_be_closed) {
        // _gs_socket_close_nolock(id);
        res = ERR_CLSD;
    } else {
        //read from slot
        slot = _gs_acquire_slot(GS_CMD_QIRD, NULL, 64, GS_TIMEOUT * 10, 1);
        _gs_send_at(GS_CMD_QIRD, "=i", id);
        if (!_gs_wait_for_buffer_mode()) {
            //oops, timeout
            res = ERR_TIMEOUT;
        }
        nargs = _gs_parse_command_arguments(slot->resp, slot->eresp, "isi", &rd, &remote_ip, &remote_len, &remote_port);
        printf("READ NARGS %i %i %i %i\n", nargs, rd, remote_len, remote_port);
        if (nargs == 3) {
            remote_len = MIN(15,(remote_len-2));
            res = zs_string_to_addr(remote_ip+1, remote_len, addr);
            addr->sin_port = OAL_GET_NETPORT(remote_port);
            if (!res) {
                //if no error in converting string to addr, assign res the read bytes count
                res = MIN(rd, len);
            } else {
                res = -1;
            }
            //if buf not large enough, exceeding data is discarded
            _gs_exit_from_buffer_mode_r(buf, len, rd, NULL);
        } else {
            if (nargs > 1 && rd == 0 && remote_len == 0) {
                //no data
                res = 0;
            } else {
                res = -1;
            }
            _gs_exit_from_buffer_mode_r(NULL, 0, 0, NULL);
        }
        _gs_wait_for_slot();
        if (slot->err) {
            res = -1;
        }
        _gs_release_slot(slot);
    }

    vosSemSignal(sock->lock);
    if (res == 0) {
        //no data arrived, wait with timeout
        printf("Waiting for rx\n");
        vosSemWaitTimeout(sock->rx, TIME_U(5000, MILLIS));
        //also check network status
        if (IS_NETWORK_UNREGISTERED_SINCE_TOO_LONG()){
            printf("closing socket forcibly from recvfrom\n");
            _gs_socket_closing(id);
        }
    }
    return res;
}

int _gs_sock_copy(int id, uint8_t* buf, int len)
{
    GSocket* sock;
    int i, rd;
    sock = &gs_sockets[id];

    printf("Sock copy\n");
    rd = 0;
    if (sock->len > 0) {
        rd = MIN(sock->len, len);
        printf("COPY %i from %i to %i/%i\n", rd, sock->head, (sock->head + rd) % MAX_SOCK_RX_BUF, sock->len);
        for (i = 0; i < rd; i++) {
            *buf++ = sock->rxbuf[sock->head];
            sock->head = (sock->head + 1) % MAX_SOCK_RX_BUF;
        }
        sock->len -= rd;
        if (!sock->len) {
            //reset buf
            sock->head = 0;
            sock->len = 0;
        }
    }
    return rd;
}

int _gs_socket_recv(int id, uint8_t* buf, int len)
{
    int trec = 0;
    int res = len;
    int rd;
    int inbuf=0;
    GSSlot* slot;
    GSocket* sock;

    sock = &gs_sockets[id];
    vosSemWait(sock->lock);
    CHECK_SOCKET_OPEN(sock);
    //read first the leftover from socket rx buffer
recv_from_buf:
    rd = _gs_sock_copy(id, buf, len);
    if (rd > 0) {
        //skip command
        res = rd;
    } else {
        printf("Check remaining\n");
        int avail = _gs_socket_available_nolock(id);
        if(avail<=0) {
            //if we are here there is no data in buffer and socket needs to be closed
            if (sock->to_be_closed) {
                // _gs_socket_close_nolock(id);
                res = ERR_CLSD;
            } else {
                res=0;
            }
        } else {
            res=0;
            //read from slot
            trec = MAX_SOCK_RX_LEN;
            if (sock->secure) {
                //avail is > 0, so for ssl, data is now in the socket buffer
                goto recv_from_buf;
                // slot = _gs_acquire_slot(GS_CMD_QSSLRECV, NULL, 64, GS_TIMEOUT * 10, 1);
                // _gs_send_at(GS_CMD_QSSLRECV, "=i,i", id, trec);
            } else {
                slot = _gs_acquire_slot(GS_CMD_QIRD, NULL, 64, GS_TIMEOUT * 10, 1);
                _gs_send_at(GS_CMD_QIRD, "=i,i", id, trec);
            }
            if (!_gs_wait_for_buffer_mode()) {
                //oops, timeout
                res = ERR_TIMEOUT;
            }
            if (_gs_parse_command_arguments(slot->resp, slot->eresp, "i", &rd) == 1) {
                //get len bytes and leave the rest in the buffer
                //if available bytes in the modem buffer are more than the read bytes
                //we need to inform the rx semaphore to avoid useless waiting
                res = MIN(len, rd);
                if (avail>rd) {
                    //trigger another
                    _gs_socket_pending(id);
                }
                _gs_exit_from_buffer_mode_r(buf, res, rd, sock);
                if(sock->proto==IPPROTO_UDP){
                    printf("udp: empty the buffer\n");
                    sock->head=0;
                    sock->len=0;
                }
            } else {
                res = ERR_IF;
                _gs_exit_from_buffer_mode_r(NULL, 0, 0, NULL);
            }
            _gs_wait_for_slot();
            if (slot->err) {
                res = ERR_IF;
            }
            _gs_release_slot(slot);
        }
    }
    inbuf=sock->len;
    vosSemSignal(sock->lock);
    if (res >= 0 && len > res && inbuf==0) {

        //due to the difference between socket_available between SSL and TCP, we need to check again here for availability
        //for secure sockets...
        if (sock->secure) { //<--- out of lock, but it's ok
            if (_gs_socket_available(id)>0) {
                _gs_socket_pending(id);
            }
        }
        //need more data, wait with timeout
        printf("Waiting for rx\n");
        if (vosSemWaitTimeout(sock->rx, TIME_U(KEEPALIVE_PERIOD, MILLIS)) == VRES_TIMEOUT) {
            //timeout without incoming data
            //it's a good place to send check for keepalives or check network reg status
            printf("Keepalive check\n");
            if ((!_gs_socket_isalive(id))||(IS_NETWORK_UNREGISTERED_SINCE_TOO_LONG())) {
                //oops, not alive!
                printf("closing socket forcibly from recv\n");
                _gs_socket_closing(id);
            }
        }
    }
    return res;
}

int _gs_socket_available(int id){
    GSocket* sock;
    int res;
    sock = &gs_sockets[id];
    vosSemWait(sock->lock);
    CHECK_SOCKET_OPEN(sock);
    res = _gs_socket_available_nolock(id);
    vosSemSignal(sock->lock);
    return res;
}

int _gs_socket_available_nolock(int id)
{
    int trec = 0;
    int res = 0;
    int total, rd, toberd;
    GSSlot* slot;
    GSocket* sock;

    sock = &gs_sockets[id];
    //read from buffer
    if (sock->len > 0){
        res = sock->len;
//WATCH OUT: the following 2 lines were commented out
    } else if (sock->to_be_closed) {
        res = ERR_CLSD;
    } else {
        if (sock->secure) {
            //QSSLRECV id,0 is not supported -_-
            //we can try to read a byte here and put it in the socket queue. It will be read later
            slot = _gs_acquire_slot(GS_CMD_QSSLRECV, NULL, 64, GS_TIMEOUT * 10, 1);
            _gs_send_at(GS_CMD_QSSLRECV, "=i,i", id,MAX_SOCK_RX_LEN);
            if (!_gs_wait_for_buffer_mode()) {
                //oops, timeout
                res = ERR_TIMEOUT;
            }
            if (_gs_parse_command_arguments(slot->resp, slot->eresp, "i", &rd) == 1) {
                if (rd>0) {
                    //read one char
                    sock->head=0;
                    sock->len=0;
                    printf("reading ssl buf %i\n",rd);
                    _gs_exit_from_buffer_mode_r(sock->rxbuf, rd, rd, NULL);
                    //put it in queue, which is empty or we won't have reached here
                    sock->len=rd;
                    sock->head=0;
                    res=rd;
                } else {
                    _gs_exit_from_buffer_mode_r(NULL, 0, 0, NULL);
                    res = 0;
                }
            } else {
                res = ERR_IF;
                _gs_exit_from_buffer_mode_r(NULL, 0, 0, NULL);
            }
            
        } else {
            //TCP CASE
            slot = _gs_acquire_slot(GS_CMD_QIRD, NULL, 64, GS_TIMEOUT * 10, 1);
            _gs_send_at(GS_CMD_QIRD, "=i,0", id);
        
            if (!_gs_wait_for_buffer_mode()) {
                //oops, timeout
                res = ERR_TIMEOUT;
            } else {
                if (_gs_parse_command_arguments(slot->resp, slot->eresp, "iii", &total, &rd, &toberd) == 3) {
                    res = toberd;
                } else {
                    res = -1;
                }
                _gs_exit_from_buffer_mode_r(NULL, 0, 0, NULL);
            }
        }

        _gs_wait_for_slot();
        if (slot->err) {
            res = ERR_IF;
        }
        _gs_release_slot(slot);
    }
    if (res==0 && sock->to_be_closed) {
        return ERR_CLSD;
    }
    return res;
}

void _gs_socket_closing(int id)
{
    GSocket* sock;
    sock = &gs_sockets[id];

    // vosSemWait(sock->lock);
    sock->to_be_closed = 1;
    vosSemSignal(sock->rx);
    // vosSemSignal(sock->lock);
}

void _gs_socket_pending(int id)
{
    GSocket* sock;
    sock = &gs_sockets[id];
    vosSemSignal(sock->rx);
    vosSemSignal(gs.selectlock);
}

int _gs_resolve(uint8_t* url, int len, uint8_t* addr)
{
    int res = 0, cnt;
    GSSlot* slot;
    if (IS_NETWORK_UNREGISTERED_SINCE_TOO_LONG()){
        printf("can't resolve, no network\n");
        return res;
    }

    vosSemWait(gs.dnsmode);
    gs.dns_ready = 0;
    slot = _gs_acquire_slot(GS_CMD_QIDNSGIP, NULL, 0, GS_TIMEOUT * 60, 0);
    _gs_send_at(GS_CMD_QIDNSGIP, "=i,\"s\"", GS_PROFILE, url, len);
    _gs_wait_for_slot();
    if (slot->err) {
        printf("SLOT ERROR\n");
        res = -1;
    }
    for (cnt = 0; cnt < 150; cnt++) {
        //wait at most 15s to resolve: max is 60s but the command often hangs
        vosThSleep(TIME_U(100, MILLIS));
        if (gs.dns_ready)
            break;
    }

    if (gs.dns_ready) {
        res = gs.dnsaddrlen; //0 in case of error
        printf("copying from %x to %x %i bytes\n", gs.dnsaddr, addr, res);
        memcpy(addr, gs.dnsaddr, res);
    } else {
        printf("DNS NOT READY\n");
        res = -1;
    }
    _gs_release_slot(slot);
    vosSemSignal(gs.dnsmode);
    return res;
}


/**
 * @brief Retrieve the list of operators with +COPS test command
 *
 * If successful, stores the retrieved operators with their parameters
 * in the global operator list (capped at MAX_OPS) and set their number (gsopn) accordingly
 *
 * @return 0 on success
 */
int _gs_list_operators(void)
{
    GSSlot* slot;
    int err;
    slot = _gs_acquire_slot(GS_CMD_COPS, NULL, MAX_CMD, GS_TIMEOUT * 60, 1);
    _gs_send_at(GS_CMD_COPS, "=?");
    _gs_wait_for_slot();
    if (slot->err) {
        err = slot->err;
        _gs_release_slot(slot);
        return err;
    }
    uint8_t* buf = slot->resp;
    uint8_t nops = 0, nres, nt = 0;
    while (buf < slot->eresp) {
        printf("start buf %x\n", *buf);
        if (!(*buf == '(' && *(buf + 3) == '"'))
            break; //not a good record
        buf++;     //skip (
        gsops[nops].type = *buf - '0';
        buf++;
        buf++;
        buf++; //skip ,"
        nt = 0;
        while (*buf != '"') {
            gsops[nops].fmt_long[nt++] = *buf++;
        }
        gsops[nops].fmtl_l = nt;
        buf++;
        buf++;
        buf++; //skip ","
        nt = 0;
        while (*buf != '"') {
            gsops[nops].fmt_short[nt++] = *buf++;
        }
        gsops[nops].fmts_l = nt;
        buf++;
        buf++;
        buf++; //skip ","
        nt = 0;
        while (*buf != '"') {
            gsops[nops].fmt_code[nt++] = *buf++;
        }
        gsops[nops].fmtc_l = nt;
        //skip up to )
        while ((buf < slot->eresp) && (*buf != ')')) {
            printf("Skip %x\n", *buf);
            buf++;
        }
        buf++;
        if (*buf == ',')
            buf++; //skip the comma if present
        printf("Last %x\n", *buf);
        nops++;
        if (nops == MAX_OPS)
            break;
    }
    gsopn = nops;
    _gs_release_slot(slot);
    return 0;
}
int _gs_set_operator(uint8_t* operator, int oplen)
{
    GSSlot* slot;
    int err;
    slot = _gs_acquire_slot(GS_CMD_COPS, NULL, MAX_CMD, GS_TIMEOUT * 60, 0);
    _gs_send_at(GS_CMD_COPS, "=1,1,\"s\"", operator, oplen);
    _gs_wait_for_slot();
    err = slot->err;
    _gs_release_slot(slot);
    return err;
}

void _gs_update_network_status(uint8_t *s0, int l0, uint8_t *s1, int l1)
{
    gs.tech = 0; // start with none
    if (gs.eps_status >= GS_REG_OK) {
        if (gs.eps_act == 8)
            gs.tech |= GS_RAT_LTE_M1;
        else if (gs.eps_act == 9)
            gs.tech |= GS_RAT_LTE_NB1;
        else
            gs.tech |= GS_RAT_LTE;
    }
    if (gs.gprs_status >= GS_REG_OK)
        gs.tech |= GS_RAT_GPRS; // add GPRS
    if (gs.gsm_status >= GS_REG_OK)
        gs.tech |= GS_RAT_GSM; // add GSM

    if (gs.tech == 0) {
        // neither GSM nor GPRS network is registered
        memset(gs.lac, 0, 10);
        memset(gs.ci, 0, 10);
    } else if (s0 != NULL && s1 != NULL && l0 > 0 && l1 > 0) {
        l0 = MIN(9, l0);
        memcpy(gs.lac, s0, l0);
        gs.lac[l0] = 0;
        l1 = MIN(9, l1);
        memcpy(gs.ci, s1, l1);
        gs.ci[l1] = 0;
    }

    // update network status (data connection)
    int was_registered = gs.registered;
    if (gs.tech & (GS_RAT_LTE|GS_RAT_LTE_M1|GS_RAT_LTE_NB1))
        gs.registered = gs.eps_status;
    else if (gs.tech & GS_RAT_GPRS)
        gs.registered = gs.gprs_status;
    else
        gs.registered = GS_REG_NOT;

    // printf("status is %i %i %i\n",p1,gs.registered,was_registered);

    //Check registration status. With creg==0 or creg==2, we have no pdp even if no pdpdeact urc is issued.
    //Get the time of network status change and if it stays unreg too long, disconnect all sockets.
    //The behaviour upon losing network seems to be a +creg=2 urc, followed by a long time of retrying
    //to get the network. After some time (quite long), a creg=0 is issued followed by alternating creg=2 and creg=0.
    if (gs.registered >= GS_REG_OK && was_registered < GS_REG_OK) {
        // printf("SET REGISTRATION STATUS TIME of reg\n");
        gs.registration_status_time = (uint32_t)(vosMillis() / 1000);

    } else if (gs.registered < GS_REG_OK && was_registered >= GS_REG_OK) {
        //it is now registered but it wasn't before or viceversa: set registration status time in seconds
        // printf("SET REGISTRATION STATUS TIME of unreg\n");
        gs.registration_status_time = (uint32_t)(vosMillis() / 1000);
    }

    //THIS IS NOT NEEDED: currently open sockets will close automatically
    // if (IS_NETWORK_UNREGISTERED_SINCE_TOO_LONG()) {
        // printf("closing all sockets forcibly from network status\n");
        // _gs_socket_close_all();
        //set unreg time again
        // gs.registration_status_time = (uint32_t)(vosMillis() / 1000);
    // }
}

static const uint8_t reg_status[6] = {
    GS_REG_NOT, GS_REG_OK, GS_REG_SEARCH, GS_REG_DENIED, GS_REG_UNKNOWN, GS_REG_ROAMING
};

int _gs_set_gsm_status_from_creg(uint8_t* buf, uint8_t* ebuf, int from_urc)
{
    int p0, p1, p2, l0, l1;
    uint8_t *s0, *s1;

    int nargs = 0;
    if (!from_urc) {
        nargs = _gs_parse_command_arguments(buf, ebuf, "iiSS", &p0, &p1, &s0, &l0, &s1, &l1);
        nargs--; // discard p0
    } else {
        nargs = _gs_parse_command_arguments(buf, ebuf, "iSS", &p1, &s0, &l0, &s1, &l1);
    }
    if (nargs < 1) return 0;
    //update gsm status
    gs.gsm_status = reg_status[p1];

    if (nargs < 3) {
        s0 = s1 = NULL;
        l0 = l1 = 0;
    }
    _gs_update_network_status(s0,l0,s1,l1);
    return 1;
}

int _gs_set_gprs_status_from_cgreg(uint8_t* buf, uint8_t* ebuf, int from_urc)
{
    int p0, p1, p2, l0, l1;
    uint8_t *s0, *s1;

    int nargs = 0;
    if (!from_urc) {
        nargs = _gs_parse_command_arguments(buf, ebuf, "iiSS", &p0, &p1, &s0, &l0, &s1, &l1);
        nargs--; // discard p0
    } else {
        nargs = _gs_parse_command_arguments(buf, ebuf, "iSS", &p1, &s0, &l0, &s1, &l1);
    }
    if (nargs < 1) return 0;
    //update gprs status
    gs.gprs_status = reg_status[p1];

    if (nargs < 3) {
        s0 = s1 = NULL;
        l0 = l1 = 0;
    }
    _gs_update_network_status(s0,l0,s1,l1);
    return 1;
}

int _gs_set_eps_status_from_cereg(uint8_t* buf, uint8_t* ebuf, int from_urc)
{
    int p0, p1, p2, l0, l1;
    uint8_t *s0, *s1;

    int nargs = 0;
    if (!from_urc) {
        nargs = _gs_parse_command_arguments(buf, ebuf, "iiSSi", &p0, &p1, &s0, &l0, &s1, &l1, &p2);
        nargs--; // discard p0
    } else {
        nargs = _gs_parse_command_arguments(buf, ebuf, "iSSi", &p1, &s0, &l0, &s1, &l1, &p2);
    }
    if (nargs < 1) return 0;
    //update eps status
    gs.eps_status = reg_status[p1];

    if (nargs < 3) {
        s0 = s1 = NULL;
        l0 = l1 = 0;
    }
    if (nargs < 4)
        gs.eps_act = 1;
    else
        gs.eps_act = p2;

    _gs_update_network_status(s0,l0,s1,l1);
    return 1;
}

/**
 * @return 0 on failure (all commands)
 */
int _gs_check_network(void)
{
    GSSlot* slot;
    int res = 0;

    slot = _gs_acquire_slot(GS_CMD_CREG, NULL, 64, GS_TIMEOUT * 5, 1);
    _gs_send_at(GS_CMD_CREG, "?");
    _gs_wait_for_slot();
    res |= _gs_set_gsm_status_from_creg(slot->resp, slot->eresp, 0);
    _gs_release_slot(slot);

    slot = _gs_acquire_slot(GS_CMD_CGREG, NULL, 64, GS_TIMEOUT * 5, 1);
    _gs_send_at(GS_CMD_CGREG, "?");
    _gs_wait_for_slot();
    res |= _gs_set_gprs_status_from_cgreg(slot->resp, slot->eresp, 0);
    _gs_release_slot(slot);

    slot = _gs_acquire_slot(GS_CMD_CEREG, NULL, 64, GS_TIMEOUT * 5, 1);
    _gs_send_at(GS_CMD_CEREG, "?");
    _gs_wait_for_slot();
    res |= _gs_set_eps_status_from_cereg(slot->resp, slot->eresp, 0);
    _gs_release_slot(slot);

    return res;
}

/**
 * @brief Generalize sending AT commands for activating/disactivating PSD
 *
 * @param[in] activate  1 for activation, 0 for deactivation
 *
 * @return 0 on failure
 */
int _gs_control_psd(int activate)
{
    GSSlot* slot;
    int res;
    activate = (activate) ? 1 : 0;
    if (activate) {
        slot = _gs_acquire_slot(GS_CMD_QIACT, NULL, 0, GS_TIMEOUT * 60 * 3, 0);
        _gs_send_at(GS_CMD_QIACT, "=i", GS_PROFILE);
        _gs_wait_for_slot();
        res = !slot->err;
        _gs_release_slot(slot);
    } else {
        slot = _gs_acquire_slot(GS_CMD_QIDEACT, NULL, 0, GS_TIMEOUT * 60 * 3, 0);
        _gs_send_at(GS_CMD_QIDEACT, "=i", GS_PROFILE);
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
int _gs_configure_psd(uint8_t* apn, int apnlen, uint8_t* username, int ulen, uint8_t* pwd, int pwdlen, int auth)
{
    GSSlot* slot;
    int res;
    // slot = _gs_acquire_slot(GS_CMD_CGDCONT,NULL,0,GS_TIMEOUT,0);
    // //set context id as undefined
    // _gs_send_at(GS_CMD_CGDCONT,"=i",GS_PROFILE);
    // _gs_wait_for_slot();
    // int res = !slot->err;
    // _gs_release_slot(slot);
    // if (!res) return res;

    //configure TCP/IP PSD with IPV4IPV6
    slot = _gs_acquire_slot(GS_CMD_QICSGP, NULL, 0, GS_TIMEOUT, 0);
    _gs_send_at(GS_CMD_QICSGP, "=i,i,\"s\",\"s\",\"s\",i", GS_PROFILE, 1, apn, apnlen, username, ulen, pwd, pwdlen, auth);
    _gs_wait_for_slot();
    res = !slot->err;
    _gs_release_slot(slot);

    return res;
}

int _gs_modem_functionality(int fun)
{
    GSSlot* slot;
    slot = _gs_acquire_slot(GS_CMD_CFUN, NULL, 64, GS_TIMEOUT * 15, 0);
    _gs_send_at(GS_CMD_CFUN, "=i", fun);
    _gs_wait_for_slot();
    int res = !slot->err;
    _gs_release_slot(slot);
    return res;
}

int _gs_get_rtc(uint8_t* time)
{
    GSSlot* slot;
    uint8_t* s0;
    int l0;
    int res;
    slot = _gs_acquire_slot(GS_CMD_CCLK, NULL, 32, GS_TIMEOUT, 1);
    _gs_send_at(GS_CMD_CCLK, "?");
    _gs_wait_for_slot();
    res = !slot->err;
    if (res) {
        if (_gs_parse_command_arguments(slot->resp, slot->eresp, "s", &s0, &l0) != 1) {
            res = 0;
        } else {
            memcpy(time, s0 + 1, 20);
        }
    }
    _gs_release_slot(slot);

    return res;
}

int _gs_rssi(void)
{
    GSSlot* slot;
    int rssi = 99, ber;
    slot = _gs_acquire_slot(GS_CMD_CSQ, NULL, 32, GS_TIMEOUT, 1);
    _gs_send_at(GS_CMD_CSQ, "");
    _gs_wait_for_slot();
    if (!slot->err) {
        if (_gs_parse_command_arguments(slot->resp, slot->eresp, "ii", &rssi, &ber) != 2) {
            rssi = 99;
        }
    }
    _gs_release_slot(slot);

    return rssi;
}

int _gs_is_attached(void)
{
    int status = 0;
    // Read status of APN connection
    int p0, p1;
    GSSlot* slot;
    slot = _gs_acquire_slot(GS_CMD_QIACT, NULL, 64, GS_TIMEOUT, 1);
    _gs_send_at(GS_CMD_QIACT, "?");
    _gs_wait_for_slot();
    if (!slot->err) {
        *slot->eresp = 0;
        if (_gs_parse_command_arguments(slot->resp, slot->eresp, "ii", &p0, &p1) != 2) {
            status = 0;
        } else {
            status = p1;
        }
    }
    _gs_release_slot(slot);
    gs.attached = status;
    return status;
}

int _gs_imei(uint8_t* imei)
{
    int res = -1;
    int l0;
    uint8_t* s0 = NULL;
    GSSlot* slot;
    slot = _gs_acquire_slot(GS_CMD_GSN, NULL, 32, GS_TIMEOUT, 1);
    _gs_send_at(GS_CMD_GSN, "");
    _gs_wait_for_slot();
    if (!slot->err) {
        if (_gs_parse_command_arguments(slot->resp, slot->eresp, "s", &s0, &l0) != 1) {
            res = 0;
        } else if (s0) {
            res = MIN(16, l0);
            memcpy(imei, s0, res);
        }
    }
    _gs_release_slot(slot);
    return res;
}

int _gs_iccid(uint8_t* iccid)
{
    int res = -1;
    int l0;
    uint8_t* s0 = NULL;
    GSSlot* slot;
    slot = _gs_acquire_slot(GS_CMD_QCCID, NULL, 32, GS_TIMEOUT, 1);
    _gs_send_at(GS_CMD_QCCID, "");
    _gs_wait_for_slot();
    if (!slot->err) {
        if (_gs_parse_command_arguments(slot->resp, slot->eresp, "s", &s0, &l0) != 1) {
            res = 0;
        } else if (s0) {
            res = MIN(22, l0);
            memcpy(iccid, s0, res);
        }
    }
    _gs_release_slot(slot);
    return res;
}

int _gs_dns(uint8_t* dns)
{
    int res = -1;
    int l0, p0;
    uint8_t* s0 = NULL;
    GSSlot* slot;
    slot = _gs_acquire_slot(GS_CMD_QIDNSCFG, NULL, 64, GS_TIMEOUT, 1);
    _gs_send_at(GS_CMD_QIDNSCFG, "=i", GS_PROFILE);
    _gs_wait_for_slot();
    if (!slot->err) {
        *slot->eresp = 0;
        if (_gs_parse_command_arguments(slot->resp, slot->eresp, "iS", &p0, &s0, &l0) != 2) {
            res = 0;
        } else if (s0) {
            res = MIN(15, l0);
            memcpy(dns, s0, res);
        }
    }
    _gs_release_slot(slot);
    return res;
}

int _gs_local_ip(uint8_t* ip)
{
    int res = -1;
    int l0, p0, p1, p2;
    uint8_t* s0 = NULL;
    GSSlot* slot;
    slot = _gs_acquire_slot(GS_CMD_QIACT, NULL, 64, GS_TIMEOUT, 1);
    _gs_send_at(GS_CMD_QIACT, "?");
    _gs_wait_for_slot();
    if (!slot->err) {
        *slot->eresp = 0;
        if (_gs_parse_command_arguments(slot->resp, slot->eresp, "iiiS", &p0, &p1, &p2, &s0, &l0) != 4) {
            res = 0;
        } else if (s0) {
            res = MIN(15, l0);
            memcpy(ip, s0, res);
        }
    }
    _gs_release_slot(slot);
    return res;
}

int _gs_cell_info(int* mcc, int* mnc)
{
    int res = -1;
    int p0, l0, l1, l2, l3;
    uint8_t *s0 = NULL, *s1 = NULL, *s2 = NULL, *s3 = NULL, *s4 = NULL;
    GSSlot* slot;
    slot = _gs_acquire_slot(GS_CMD_QENG, NULL, 256, 5 * GS_TIMEOUT, 1);
    _gs_send_at(GS_CMD_QENG, "=s", "\"servingcell\"", 13);
    _gs_wait_for_slot();
    if (!slot->err) {
        if (_gs_parse_command_arguments(slot->resp, slot->eresp,
                "SSS", &s0, &l0, &s1, &l1, &s2, &l2) == 3) {
            if (memcmp("GSM", s2, 3) == 0) {
                if (_gs_parse_command_arguments(slot->resp, slot->eresp,
                        "SSSii", &s0, &l0, &s1, &l1, &s2, &l2, mcc, mnc) != 5) {
                    res = 0;
                } else if (s0 && s1 && s2) {
                    res = 1;
                }
            }
            else {
                if (_gs_parse_command_arguments(slot->resp, slot->eresp,
                        "SSSSii", &s0, &l0, &s1, &l1, &s2, &l2, &s3, &l3, mcc, mnc) != 6) {
                    res = 0;
                } else if (s0 && s1 && s2) {
                    res = 1;
                }
            }
        }
        else {
            res = 0;
        }
    }
    _gs_release_slot(slot);
    return res;
}

/////////// SMS HANDLING
int _gs_sms_send(uint8_t* num, int numlen, uint8_t* txt, int txtlen)
{
    int res = -2;
    int mr = -1;
    GSSlot* slot;
    slot = _gs_acquire_slot(GS_CMD_CMGS, NULL, 64, GS_TIMEOUT * 120, 1);
    _gs_send_at(GS_CMD_CMGS, "=\"s\"", num, numlen);
    res = _gs_wait_for_slot_mode(txt, txtlen, "\x1A", 1);
    _gs_wait_for_slot();
    if (!slot->err) {
        if (_gs_parse_command_arguments(slot->resp, slot->eresp, "i", &mr) == 1) {
            res = mr;
        } else {
            res = -1;
        }
    }
    _gs_release_slot(slot);
    return res;
}

int _gs_sms_list(int unread, GSSMS* sms, int maxsms, int offset)
{
    int res = -2;
    int mr = -1;
    GSSlot* slot;
    slot = _gs_acquire_slot(GS_CMD_CMGL, NULL, 64, GS_TIMEOUT * 60, 1);
    gs.cursms = -1;
    gs.skipsms = 1;
    gs.maxsms = maxsms;
    gs.offsetsms = offset;
    gs.sms = sms;
    gs.pendingsms = 0;
    if (unread) {
        _gs_send_at(GS_CMD_CMGL, "=\"REC UNREAD\"");
    } else {
        _gs_send_at(GS_CMD_CMGL, "=\"ALL\"");
    }
    _gs_wait_for_slot();
    if (slot->err) {
        res = -1;
    } else
        res = gs.cursms + 1;
    _gs_release_slot(slot);
    return res;
}

int _gs_sms_delete(int index)
{
    int res;
    GSSlot* slot;
    slot = _gs_acquire_slot(GS_CMD_CMGD, NULL, 64, GS_TIMEOUT, 0);
    _gs_send_at(GS_CMD_CMGD, "=i", index);
    _gs_wait_for_slot();
    if (slot->err) {
        res = -1;
    } else
        res = index;
    _gs_release_slot(slot);
    return res;
}

int _gs_sms_get_scsa(uint8_t* scsa)
{
    int res = -2;
    uint8_t* sc;
    int sclen;
    GSSlot* slot;
    slot = _gs_acquire_slot(GS_CMD_CSCA, NULL, 64, GS_TIMEOUT, 1);
    _gs_send_at(GS_CMD_CSCA, "?");
    _gs_wait_for_slot();
    if (!slot->err) {
        if (_gs_parse_command_arguments(slot->resp, slot->eresp, "s", &sc, &sclen) == 1) {
            res = sclen - 2;
            memcpy(scsa, sc + 1, MIN(res, 32));
        } else {
            res = -1;
        }
    }
    _gs_release_slot(slot);
    return res;
}

int _gs_sms_set_scsa(uint8_t* scsa, int scsalen)
{
    int res = -1;
    GSSlot* slot;
    slot = _gs_acquire_slot(GS_CMD_CSCA, NULL, 64, GS_TIMEOUT, 0);
    _gs_send_at(GS_CMD_CSCA, "=\"s\"", scsa, scsalen);
    _gs_wait_for_slot();
    if (!slot->err) {
        res = 1;
    }
    _gs_release_slot(slot);
    return res;
}




///////GZSOCKETS

int bg96_gzsock_socket(int family, int type, int protocol) {
    int sock_id = -1;
    protocol = (type == SOCK_DGRAM) ? 17 : 6;
    printf("protocol %i\n",protocol);
    sock_id = _gs_socket_new(protocol,0);
    return sock_id;
}


int bg96_gzsock_connect(int sock, const struct sockaddr *addr, socklen_t addrlen) {
    return _gs_socket_connect(sock, (struct sockaddr_in *) addr);
}

int bg96_gzsock_close(int sock) {
    return _gs_socket_close(sock);
}

int bg96_gzsock_send(int sock, const void *dataptr, size_t size, int flags) {
    int32_t wrt;
    int32_t tsnd;
    int32_t err=ERR_OK;
    uint8_t *buf = (uint8_t*) dataptr;
    int len = size;
    wrt=0;
    while(wrt<len){
        tsnd = MIN(MAX_SOCK_TX_LEN,(len-wrt));
        tsnd = _gs_socket_send(sock,buf+wrt,tsnd);
        if (tsnd<0) {
            return -1;
        }
        wrt+=tsnd;
    }
    return wrt;
}

int bg96_gzsock_sendto(int sock, const void *dataptr, size_t size, int flags, const struct sockaddr *to, socklen_t tolen) {
    int32_t wrt;
    int32_t tsnd;
    int32_t err=ERR_OK;
    uint8_t *buf = (uint8_t*) dataptr;
    uint16_t len = size;
    wrt=0;
    while(wrt<len){
        tsnd = MIN(MAX_SOCK_TX_LEN,(len-wrt));
        tsnd = _gs_socket_sendto(sock,buf+wrt,tsnd,(struct sockaddr_in *) to);
        if (tsnd<0) {
            return -1;
        }
        wrt+=tsnd;
    }
    return wrt;
}

int bg96_gzsock_recv(int sock, void *mem, size_t len, int flags){
    int rb;
    int trec;
    uint8_t *buf = (uint8_t*) mem;

    rb=0;
    while(rb<len){
        trec = _gs_socket_recv(sock,buf+rb,len-rb);
        if(trec<0) {
            // when closed return data already read if any
            if (trec==ERR_CLSD && rb > 0) return rb;
            // otherwise error
            return trec;
        }
        rb+=trec;
    }
    return rb;
}

int bg96_gzsock_recvfrom(int sock, void *mem, size_t len, int flags, struct sockaddr *from, socklen_t *fromlen) {
    int rb;
    int trec;
    uint8_t *buf = (uint8_t*) mem;

    rb=0;
    while(rb<len){
        trec = _gs_socket_recvfrom(sock,buf+rb,len-rb,(struct sockaddr_in *) from);
        if (trec==0 && rb==0){
            //no data yet
            continue;
        } else if(trec<0) {
            if (trec==ERR_CLSD) return rb;
            return trec;
        } else {
            //got packet
            rb+=trec;
            break;
        }
    }
    return rb;
}

int bg96_gzsock_select(int maxfdp1, void *readset, void *writeset, void *exceptset, struct timeval *tv) {
    uint64_t tstart;
    int32_t timeout;
    uint32_t timepast;
    int32_t rdy = 0, sock = -1,r;
    fd_set *read_fds = (fd_set*) readset;
    fd_set *write_fds = (fd_set*) writeset;
    fd_set *exc_fds = (fd_set*) exceptset;

    tstart = vosMillis();
    timeout = (tv) ? ((tv->tv_sec*1000)+(tv->tv_usec/1000)):(-1);
    while(1){
        //let's try to avoid useless at commands
        //enter immediately in the waiting loop
        sock++;
        if(sock>=maxfdp1) {
            if (rdy) return rdy;
            //reset and check timeout
            sock=0;
            if (timeout>=0) {
                timepast = (uint32_t)(vosMillis()-tstart);
                if(timepast>timeout) {
                    //timeout expired
                    return 0;
                } else {
                    //let's wait
                    if(vosSemWaitTimeout(gs.selectlock,TIME_U((timeout-timepast),MILLIS))==VRES_TIMEOUT){
                        //timeout passed
                        printf("SELECT TIMEOUT EXPIRED\n");
                        return 0;
                    } else {
                        printf("SELECT SEM SIGNALED\n");
                        //go on and check sockets
                    }
                }
            } else {
                vosSemWait(gs.selectlock);
            }
            // if(timeout>=0 && ((vosMillis()-tstart)>timeout)) return 0;
            // vosSemWaitTimeout(gs.selectlock,
            // vosThSleep(TIME_U(100,MILLIS)); //sleep a bit
            //TODO: consider using RD URC to signal data ready for each socket
            //and suspend here, avoiding polling (quite cumbersome tough)
        }
        if(sock>=0&&sock<MAX_SOCKS){
            r = _gs_socket_available(sock);
            if (r>0 || r==ERR_CLSD) {
                if(read_fds) {
                    FD_SET(sock, read_fds);
                    rdy++;
                }
            } else {
                //other return codes
                if(r==ERR_CONN) {
                    //the socket does not exist
                    //this is an error (man 2 select reports a EBADF in errno)
                    //but we are good with a -1 :)
                    return -1;
                }
            }
        }
    }
    return 0;
}

int bg96_gzsock_read(int sock_id, void *mem, size_t len) {
    return bg96_gzsock_recv(sock_id, mem, len, 0);
}

int bg96_gzsock_write(int sock_id, const void *dataptr, size_t size) {
    return bg96_gzsock_send(sock_id, dataptr, size, 0);
}

int bg96_gzsock_fcntl(int s, int cmd, int val) {
    if (cmd != F_GETFL) {
        return -1;
    }
    return O_NONBLOCK;
}

int bg96_gzsock_ioctl(int s, long cmd, void *argp) {
    printf("bg96_gzsock_ioctl\n");
    return 0;
}

int bg96_gzsock_inet_addr(const char *cp) {
    printf("bg96_gzsock_inet_addr\n");
    return 0;
}

int bg96_gzsock_inet_ntoa(struct in_addr *in) {
    printf("bg96_gzsock_inet_ntoa\n");
    return 0;
}

int bg96_gzsock_accept(int s, struct sockaddr *addr, socklen_t *addrlen) {
    printf("bg96_gzsock_accept\n");
    return 0;
}

int bg96_gzsock_listen(int s, int backlog) {
    printf("bg96_gzsock_listen\n");
    return 0;
}

int bg96_gzsock_shutdown(int s, int how) {
    return 0;
}

int bg96_gzsock_getaddrinfo(const char *node, const char* service, const struct addrinfo *hints, struct addrinfo **res) {
    int32_t saddrlen;
    uint8_t saddr[16];
    int ret;
    struct sockaddr_in addr;
    //hints is ignored
    saddrlen = _gs_resolve(node,strlen(node),saddr);
    ret = zs_string_to_addr(saddr,saddrlen,&addr);
    if (ret != ERR_OK) return ret;

    *res = gc_malloc(sizeof(struct addrinfo));
    struct sockaddr_in *addr_in = gc_malloc(sizeof(struct sockaddr_in));

    struct addrinfo *res_0 = *res;
    res_0->ai_next = NULL;
    addr_in->sin_addr.s_addr = addr.sin_addr.s_addr;
    res_0->ai_addr = addr_in;
    res_0->ai_addrlen = sizeof(addr_in);

    return ERR_OK;
}

void bg96_gzsock_freeaddrinfo(struct addrinfo *ai_res) {
    printf("bg96_gzsock_freeaddrinfo \n");
    struct addrinfo *p;
    for (p = ai_res; p != NULL; p = ai_res->ai_next) {
        gc_free(p->ai_addr);
    }
    gc_free(ai_res);
}

int bg96_gzsock_setsockopt(int sock_id, int level, int optname, const void *optval, socklen_t optlen) {
    printf("bg96_gzsock_setsockopt \n");
    // if(optname != SO_RCVTIMEO)
    //     return -1;
    // gTimeout = *((int*)optval);
    return 0;
}

int bg96_gzsock_getsockopt(int sock_id, int level, int optname, void *optval, socklen_t *optlen) {
    if (optname != SO_RCVTIMEO)
        return -1;
    // optval = (void*) gTimeout;
    return 0;
}

int bg96_gzsock_bind(int sock, const struct sockaddr *name, socklen_t namelen){
    struct sockaddr_in *addr_in = (struct sockaddr_in*)name;
    return _gs_socket_bind(sock,addr_in);
}

/////// GNSS functions

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

int _gs_gnss_init(int fix_rate, int use_uart3){
    int res=-1;
    GSSlot *slot;

    //DON'T! _gs_gnss_done();

    //disable nmea (only if not using UART3)
    slot = _gs_acquire_slot(GS_CMD_QGPSCFG,NULL,64,GS_TIMEOUT,0);
    _gs_send_at(GS_CMD_QGPSCFG,"=s,i","\"nmeasrc\"",9,use_uart3 ? 1 : 0);
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

    //turn on gnss
    slot = _gs_acquire_slot(GS_CMD_QGPS,NULL,64,GS_TIMEOUT,0);
    _gs_send_at(GS_CMD_QGPS,"=i,i,i,i,i",1,30,50,0,fix_rate);
    _gs_wait_for_slot();
    res = slot->err;
    _gs_release_slot(slot);
    if(res) return res;

    //configure UART3 output if specified
    if (use_uart3) {
        slot = _gs_acquire_slot(GS_CMD_QGPSCFG,NULL,64,GS_TIMEOUT,0);
        _gs_send_at(GS_CMD_QGPSCFG,"=s,s","\"outport\"",9,"\"uartnmea\"",10);
        _gs_wait_for_slot();
        res = slot->err;
        _gs_release_slot(slot);
        if(res) return res;
    }

    return res;
}

#define GPS_NUM(x)  ((x)-'0')
#define GPS_NUM2(x,y)  ((GPS_NUM(x)*10)+(GPS_NUM(y)))

double vatof(uint8_t *s, int32_t len, err_t *err);

int _gs_gnss_loc(GNSSLoc *loc){
    int res=-1;
    err_t err;
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
            loc->lat = vatof(s1,l1,&err);
            loc->lon = vatof(s2,l2,&err);
            loc->precision = vatof(s3,l3,&err);
            loc->alt = vatof(s4,l4,&err);
            loc->fix = GPS_NUM(s5[0]); //vatoi(s5,l5,10,&err);
            loc->cog = vatof(s6,l6,&err);
            loc->cog = (int)loc->cog+((loc->cog-(int)loc->cog)*10/60);  // from deg.min to decimal degrees
            loc->speed = vatof(s7,l7,&err);

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

