#include "zerynth.h"

#define ZERYNTH_SOCKETS
#if !defined(BG96_MAX_SOCKS)
#define MAX_SOCKS 4
#else
#define MAX_SOCKS BG96_MAX_SOCKS
#endif
#include "zerynth_sockets.h"

#define MAX_BUF 1024
#define MAX_CMD 545
// max out packet len supported by modem
#define MAX_SOCK_TX_LEN 1460
// max len of any packet read from modem
#define MAX_SOCK_RX_BUF 256
// request len for buffered reads (<=RX_BUF)
#define MAX_SOCK_RX_LEN 256
#define MAX_OPS 6
#define MAX_ERR_LEN 32
#define GS_TIMEOUT 1000

typedef struct _gsm_socket {
    uint8_t acquired;
    uint8_t proto;
    uint8_t to_be_closed;
    uint8_t secure;
    uint8_t connected;
    uint8_t bound;
    uint16_t timeout;
    VSemaphore rx;
    VSemaphore lock;
    uint8_t rxbuf[MAX_SOCK_RX_BUF];
    uint16_t head;
    uint16_t len;
} GSocket;

//COMMANDS

#define MAKE_CMD(group, command, response) (((group) << 24) | ((command) << 16) | (response))
#define DEF_CMD(cmd, response, urc, id)         \
    {                                           \
        cmd, sizeof(cmd) - 1, response, urc, id \
    }

typedef struct _gs_cmd {
    uint8_t body[16];
    uint8_t len;
    uint8_t response_type;
    uint8_t urc;
    uint8_t id;
} GSCmd;

//COMMAND SLOTS

#define MAX_SLOTS 16
typedef struct _gs_slot {
    GSCmd* cmd;
    uint8_t err;
    uint8_t allocated;
    uint8_t has_params;
    uint8_t params;
    uint16_t max_size;
    uint16_t unused2;
    uint32_t stime;
    uint32_t timeout;
    uint8_t* resp;
    uint8_t* eresp;
} GSSlot;

////////////OPERATORS

typedef struct _gs_operator {
    uint8_t type;
    uint8_t fmtl_l;
    uint8_t fmts_l;
    uint8_t fmtc_l;
    uint8_t fmt_long[24];
    uint8_t fmt_short[10];
    uint8_t fmt_code[6];
} GSOp;

typedef struct _gs_sms {
    uint8_t oaddr[16];
    uint8_t ts[24];
    uint8_t txt[160];
    uint8_t oaddrlen;
    uint8_t tslen;
    uint8_t unread;
    uint8_t txtlen;
    int index;
} GSSMS;

////////////GSM STATUS

typedef struct _gsm_status {
    uint8_t volatile initialized;
    uint8_t volatile talking;
    uint8_t volatile running;
    uint8_t attached;
    uint8_t registered;
    uint32_t registration_status_time;
    uint8_t gsm_status;
    uint8_t gprs_status;
    uint8_t eps_status;
    uint8_t eps_act;
    uint8_t errlen;
    uint8_t mode;
    uint8_t rssi;
    uint8_t serial;
    uint16_t dtr;
    uint16_t rts;
    uint16_t rx;
    uint16_t tx;
    uint16_t bytes;
    GSSlot* slot;
    VSemaphore sendlock;
    VSemaphore slotlock;
    VSemaphore slotdone;
    VSemaphore bufmode;
    VSemaphore dnsmode;
    VSemaphore selectlock;
    VThread thread;
    uint8_t errmsg[MAX_ERR_LEN];
    uint8_t buffer[MAX_BUF];
    uint8_t dnsaddr[16];
    uint8_t dnsaddrlen;
    uint8_t dns_ready;
    uint8_t dns_count;
    uint8_t lac[10];
    uint8_t ci[10];
    uint8_t tech;
    uint8_t skipsms;
    uint8_t maxsms;
    int offsetsms;
    int cursms;
    int pendingsms;
    GSSMS* sms;
} GStatus;

//DEFINES
#define GS_PROFILE 1

#define GS_ERR_OK 0
#define GS_ERR_TIMEOUT 1
#define GS_ERR_INVALID 2

// keep order, so that >= OK is registered
#define GS_REG_NOT 0
#define GS_REG_UNKNOWN 1
#define GS_REG_SEARCH 2
#define GS_REG_DENIED 3
#define GS_REG_OK 4
#define GS_REG_ROAMING 5

// Radio Access Technology (bit field)
#define GS_RAT_GSM      0x01
#define GS_RAT_GPRS     0x02
#define GS_RAT_LTE      0x04
#define GS_RAT_LTE_M1   0x08
#define GS_RAT_LTE_NB1  0x10

#define KNOWN_COMMANDS (sizeof(gs_commands) / sizeof(GSCmd))
#define GS_MIN(a) (((a) < (gs.bytes)) ? (a) : (gs.bytes))

#define GS_MODE_NORMAL 0
#define GS_MODE_PROMPT 1
#define GS_MODE_BUFFER 2

#define GS_CMD_NORMAL 1
#define GS_CMD_URC 2
#define GS_CMD_LINE 4

#define GS_MAX_NETWORK_DOWN_TIME 60

//RESPONSES
// only ok
#define GS_RES_OK 0
// one line of params, then ok
#define GS_RES_PARAM_OK 1
// no answer
#define GS_RES_STR 2
// str param, then ok
#define GS_RES_STR_OK 3

enum {
    GS_CMD_CCLK,
    GS_CMD_CEREG,
    GS_CMD_CFUN,
    GS_CMD_CGATT,
    GS_CMD_CGDCONT,
    GS_CMD_CGEREP,
    GS_CMD_CGEV,
    GS_CMD_CGREG,
    GS_CMD_CMEE,
    GS_CMD_CMGD,
    GS_CMD_CMGF,
    GS_CMD_CMGL,
    GS_CMD_CMGR,
    GS_CMD_CMGS,
    GS_CMD_CMTI,

    GS_CMD_COPS,
    GS_CMD_CPMS,
    GS_CMD_CREG,
    GS_CMD_CSCA,
    GS_CMD_CSQ,

    GS_CMD_GSN,
    GS_CMD_QCCID,
    GS_CMD_QCFG,
    
    GS_CMD_QENG,
    
    GS_CMD_QFDEL,
    GS_CMD_QFUPL,

    GS_CMD_QGPS,
    GS_CMD_QGPSCFG,
    GS_CMD_QGPSEND,
    GS_CMD_QGPSLOC,

    GS_CMD_QIACT,
    GS_CMD_QICLOSE,
    GS_CMD_QICSGP,
    GS_CMD_QIDEACT,
    GS_CMD_QIDNSCFG,
    GS_CMD_QIDNSGIP,
    GS_CMD_QIOPEN,
    GS_CMD_QIRD,
    GS_CMD_QISEND,
    GS_CMD_QIURC,

    GS_CMD_QSSLCFG,
    GS_CMD_QSSLCLOSE,
    GS_CMD_QSSLOPEN,
    GS_CMD_QSSLRECV,
    GS_CMD_QSSLSEND,
    GS_CMD_QSSLURC,
};

#define GS_GET_CMD(cmdid) (&gs_commands[cmdid])

static const GSCmd gs_commands[] = {

    DEF_CMD("+CCLK", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_CCLK),
    DEF_CMD("+CEREG", GS_RES_OK, GS_CMD_NORMAL | GS_CMD_URC, GS_CMD_CEREG),
    DEF_CMD("+CFUN", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_CFUN),

    DEF_CMD("+CGATT", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_CGATT),
    DEF_CMD("+CGDCONT", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_CGDCONT),
    DEF_CMD("+CGEREP", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_CGEREP),
    DEF_CMD("+CGEV", GS_RES_OK, GS_CMD_URC, GS_CMD_CGEV),
    DEF_CMD("+CGREG", GS_RES_OK, GS_CMD_NORMAL | GS_CMD_URC, GS_CMD_CGREG),
    DEF_CMD("+CMEE", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_CMEE),

    DEF_CMD("+CMGD", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_CMGD),
    DEF_CMD("+CMGF", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_CMGF),
    DEF_CMD("+CMGL", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_CMGL),
    DEF_CMD("+CMGR", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_CMGR),
    DEF_CMD("+CMGS", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_CMGS),
    DEF_CMD("+CMTI", GS_RES_OK, GS_CMD_URC, GS_CMD_CMTI),

    DEF_CMD("+COPS", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_COPS),
    DEF_CMD("+CPMS", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_CPMS),
    DEF_CMD("+CREG", GS_RES_OK, GS_CMD_NORMAL | GS_CMD_URC, GS_CMD_CREG),

    DEF_CMD("+CSCA", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_CSCA),

    DEF_CMD("+CSQ", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_CSQ),
    DEF_CMD("+GSN", GS_RES_STR_OK, GS_CMD_NORMAL, GS_CMD_GSN),
    DEF_CMD("+QCCID", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QCCID),
    DEF_CMD("+QCFG", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QCFG),

    DEF_CMD("+QENG", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QENG),

    DEF_CMD("+QFDEL", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QFDEL),
    DEF_CMD("+QFUPL", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QFUPL),

    DEF_CMD("+QGPS", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QGPS),
    DEF_CMD("+QGPSCFG", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QGPSCFG),
    DEF_CMD("+QGPSEND", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QGPSEND),
    DEF_CMD("+QGPSLOC", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QGPSLOC),

    DEF_CMD("+QIACT", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QIACT),
    DEF_CMD("+QICLOSE", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QICLOSE),
    DEF_CMD("+QICSGP", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QICSGP),
    DEF_CMD("+QIDEACT", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QIDEACT),
    DEF_CMD("+QIDNSCFG", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QIDNSCFG),
    DEF_CMD("+QIDNSGIP", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QIDNSGIP),
    DEF_CMD("+QIOPEN", GS_RES_OK, GS_CMD_URC, GS_CMD_QIOPEN),
    DEF_CMD("+QIRD", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QIRD),
    DEF_CMD("+QISEND", GS_RES_STR, GS_CMD_NORMAL, GS_CMD_QISEND),
    DEF_CMD("+QIURC", GS_RES_OK, GS_CMD_URC, GS_CMD_QIURC),

    DEF_CMD("+QSSLCFG", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QSSLCFG),
    DEF_CMD("+QSSLCLOSE", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QSSLCLOSE),
    DEF_CMD("+QSSLOPEN", GS_RES_OK, GS_CMD_URC, GS_CMD_QSSLOPEN),
    DEF_CMD("+QSSLRECV", GS_RES_OK, GS_CMD_NORMAL, GS_CMD_QSSLRECV),
    DEF_CMD("+QSSLSEND", GS_RES_STR, GS_CMD_NORMAL, GS_CMD_QSSLSEND),
    DEF_CMD("+QSSLURC", GS_RES_OK, GS_CMD_URC, GS_CMD_QSSLURC),

};

extern GStatus gs;
extern GSOp gsops[MAX_OPS];
extern int gsopn;

#define KEEPALIVE_PERIOD 30000
//if more than 1500 bytes are unacked in the last KEEPALIVE_PERIOD, consider connection broken
#define MAX_UNACKED_DATA 1500
#define IS_NETWORK_UNREGISTERED_SINCE_TOO_LONG() ((gs.registered == GS_REG_NOT || gs.registered == GS_REG_DENIED) && ((((uint32_t)(vosMillis() / 1000)) - gs.registration_status_time) > GS_MAX_NETWORK_DOWN_TIME))


#define CHECK_SOCKET_OPEN(sock) do { \
    if (!((sock)->connected==1 && (sock)->acquired)) { \
        vosSemSignal((sock)->lock); \
        return ERR_CONN; \
    } \
    } while(0)
            

void _gs_init(void);
int _gs_start(void);
int _gs_stop(void);
int _gs_config0(int min_fun);
void _gs_loop(void* args);
int _gs_list_operators(void);
int _gs_set_operator(uint8_t* operator, int oplen);
int _gs_check_network(void);
int _gs_control_psd(int activate);
int _gs_config_psd(void);
int _gs_configure_psd(uint8_t* apn, int apnlen, uint8_t* username, int ulen, uint8_t* pwd, int pwdlen, int auth);
int _gs_set_gsm_status_from_creg(uint8_t* buf, uint8_t* ebuf, int from_urc);
int _gs_set_gprs_status_from_cgreg(uint8_t* buf, uint8_t* ebuf, int from_urc);
int _gs_set_eps_status_from_cereg(uint8_t* buf, uint8_t* ebuf, int from_urc);
int _gs_set_rat(int rat, int band);

int _gs_modem_functionality(int fun);
int _gs_get_rtc(uint8_t* time);
int _gs_rssi(void);
int _gs_attach(int attach);
int _gs_is_attached(void);
int _gs_imei(uint8_t* imei);
int _gs_iccid(uint8_t* iccid);
int _gs_dns(uint8_t* dns);
int _gs_local_ip(uint8_t* ip);
int _gs_cell_info(int* mcc, int* mnc);

int _gs_socket_connect(int id, struct sockaddr_in *addr);
int _gs_socket_new(int proto, int secure);
int _gs_socket_send(int id, uint8_t* buf, int len);
int _gs_socket_sendto(int id, uint8_t* buf, int len, struct sockaddr_in *addr);
int _gs_socket_recv(int id, uint8_t* buf, int len);
int _gs_socket_recvfrom(int id, uint8_t* buf, int len, struct sockaddr_in *addr);
int _gs_socket_available(int id);
int _gs_socket_available_nolock(int id);
int _gs_socket_close(int id);
int _gs_resolve(uint8_t* url, int len, uint8_t* addr);
int _gs_socket_tls(int id, uint8_t* cacert, int cacertlen, uint8_t* clicert, int clicertlen, uint8_t* pvkey, int pvkeylen, int authmode);
int _gs_socket_bind(int id, struct sockaddr_in *addr);
int _gs_socket_isalive(int id);
void _gs_socket_close_all(void);

int _gs_sms_list(int unread, GSSMS* sms, int maxsms, int offset);
int _gs_sms_send(uint8_t* num, int numlen, uint8_t* txt, int txtlen);
int _gs_sms_delete(int index);
int _gs_sms_get_scsa(uint8_t* scsa);
int _gs_sms_set_scsa(uint8_t* scsa, int scsalen);

int bg96_gzsock_socket(int family, int type, int protocol);
int bg96_gzsock_connect(int sock, const struct sockaddr *addr, socklen_t addrlen);
int bg96_gzsock_setsockopt(int sock, int level, int optname, const void *optval, socklen_t optlen);
int bg96_gzsock_getsockopt(int sock, int level, int optname, void *optval, socklen_t *optlen);
int bg96_gzsock_send(int sock, const void *dataptr, size_t size, int flags);
int bg96_gzsock_sendto(int sock, const void *dataptr, size_t size, int flags, const struct sockaddr *to, socklen_t tolen);
int bg96_gzsock_write(int sock, const void *dataptr, size_t size);
int bg96_gzsock_recv(int sock, void *mem, size_t len, int flags);
int bg96_gzsock_recvfrom(int sock, void *mem, size_t len, int flags, struct sockaddr *from, socklen_t *fromlen);
int bg96_gzsock_read(int sock, void *mem, size_t len);
int bg96_gzsock_close(int sock);
int bg96_gzsock_select(int maxfdp1, void *readset, void *writeset, void *exceptset, struct timeval *tv);
int bg96_gzsock_fcntl(int s, int cmd, int val);
int bg96_gzsock_shutdown(int s, int how);
int bg96_gzsock_getaddrinfo(const char *node, const char* service, const struct addrinfo *hints, struct addrinfo **res);
void bg96_gzsock_freeaddrinfo(struct addrinfo *ai_res);
int bg96_gzsock_bind(int sock, const struct sockaddr *name, socklen_t namelen);

typedef struct _gnssloc {
    uint8_t yy;
    uint8_t MM;
    uint8_t dd;
    uint8_t hh;
    uint8_t mm;
    uint8_t ss;
    uint8_t fix;
    uint8_t nsat;
    double lat;
    double lon;
    double alt;
    double speed;
    double cog;
    double precision;
} GNSSLoc;

int _gs_gnss_init(int fix_rate, int use_uart3);
int _gs_gnss_done(void);
int _gs_gnss_loc(GNSSLoc *loc);
