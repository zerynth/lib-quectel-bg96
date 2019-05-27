#define MAX_BUF 1024
#define MAX_CMD 545
#if !defined(BG96_MAX_SOCKS)
#define MAX_SOCKS 4
#else
#define MAX_SOCKS BG96_MAX_SOCKS
#endif
#define MAX_SOCK_BUF 512
#define MAX_SOCK_RX_BUF 1500
#define MAX_OPS   6
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

#define MAKE_CMD(group,command,response) (((group)<<24)|((command)<<16)|(response))
#define DEF_CMD(cmd,response,urc,id)  {cmd,sizeof(cmd)-1,response,urc,id}

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
    GSCmd *cmd;
    uint8_t err;
    uint8_t allocated;
    uint8_t has_params;
    uint8_t params;
    uint16_t max_size;
    uint16_t unused2;
    uint32_t stime;
    uint32_t timeout;
    uint8_t *resp;
    uint8_t *eresp;
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
}GSOp;


////////////GSM STATUS

typedef struct _gsm_status{
    uint8_t initialized;
    uint8_t attached;
    uint8_t registered;
    int8_t secure_sock_id;
    uint8_t gprs;
    uint8_t gprs_mode;
    uint8_t errlen;
    uint8_t mode;
    uint8_t rssi;
    uint8_t serial;
    uint16_t dtr;
    uint16_t rts;
    uint16_t rx;
    uint16_t tx;
    uint16_t poweron;
    uint16_t reset;
    uint16_t status;
    uint16_t bytes;
    GSSlot *slot;
    VSemaphore sendlock;
    VSemaphore slotlock;
    VSemaphore slotdone;
    VSemaphore bufmode;
    VSemaphore dnsmode;
    VThread thread;
    uint8_t errmsg[MAX_ERR_LEN];
    uint8_t buffer[MAX_CMD];
    uint8_t dnsaddr[16];
    uint8_t dnsaddrlen;
    uint8_t dns_ready;
    uint8_t dns_count;
    uint8_t lac[4];
    uint8_t ci[4];
    uint8_t tech;

} GStatus;

//DEFINES
#define GS_PROFILE 1

#define GS_ERR_OK      0
#define GS_ERR_TIMEOUT 1


#define GS_REG_DENIED  2
#define GS_REG_NOT     0
#define GS_REG_OK      1
#define GS_REG_ROAMING 3

#define KNOWN_COMMANDS (sizeof(gs_commands)/sizeof(GSCmd))
#define GS_MIN(a)   (((a)<(gs.bytes)) ? (a):(gs.bytes))

#define GS_MODE_NORMAL 0
#define GS_MODE_PROMPT 1
#define GS_MODE_BUFFER 2

#define GS_CMD_NORMAL 0
#define GS_CMD_URC    1 
#define GS_CMD_LINE   2

//RESPONSES
// only ok
#define GS_RES_OK        0
// one line of params, then ok
#define GS_RES_PARAM_OK  1
// no answer
#define GS_RES_STR        2
// str param, then ok
#define GS_RES_STR_OK        3

#define GS_CMD_CCLK             0
#define GS_CMD_CGATT            1
#define GS_CMD_CGDCONT          2
#define GS_CMD_CMEE             3
#define GS_CMD_COPS             4
#define GS_CMD_CREG             5
#define GS_CMD_CSQ              6

#define GS_CMD_GSN              7
#define GS_CMD_QCCID            8
#define GS_CMD_QCFG             9

#define GS_CMD_QFDEL            10
#define GS_CMD_QFUPL            11

#define GS_CMD_QGPS             12
#define GS_CMD_QGPSCFG          13
#define GS_CMD_QGPSEND          14
#define GS_CMD_QGPSLOC          15

#define GS_CMD_QIACT            16
#define GS_CMD_QICLOSE          17
#define GS_CMD_QICSGP           18
#define GS_CMD_QIDEACT          19
#define GS_CMD_QIDNSCFG         20
#define GS_CMD_QIDNSGIP         21
#define GS_CMD_QIOPEN           22
#define GS_CMD_QIRD             23
#define GS_CMD_QISEND           24
#define GS_CMD_QIURC            25

#define GS_CMD_QSSLCFG          26
#define GS_CMD_QSSLCLOSE        27
#define GS_CMD_QSSLOPEN         28
#define GS_CMD_QSSLRECV         29
#define GS_CMD_QSSLSEND         30
#define GS_CMD_QSSLURC          31

#define GS_GET_CMD(cmdid) (&gs_commands[cmdid])

static const GSCmd gs_commands[] = {

    DEF_CMD("+CCLK",      GS_RES_OK, GS_CMD_NORMAL , GS_CMD_CCLK),
    DEF_CMD("+CGATT",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_CGATT),
    DEF_CMD("+CGDCONT",   GS_RES_OK, GS_CMD_NORMAL , GS_CMD_CGDCONT),
    DEF_CMD("+CMEE",      GS_RES_OK, GS_CMD_NORMAL , GS_CMD_CMEE),
    DEF_CMD("+COPS",      GS_RES_OK, GS_CMD_NORMAL , GS_CMD_COPS),
    DEF_CMD("+CREG",      GS_RES_OK, GS_CMD_NORMAL , GS_CMD_CREG),
    DEF_CMD("+CSQ",       GS_RES_OK, GS_CMD_NORMAL , GS_CMD_CSQ),
    DEF_CMD("+GSN",       GS_RES_STR_OK, GS_CMD_NORMAL , GS_CMD_GSN),
    DEF_CMD("+QCCID",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QCCID),
    DEF_CMD("+QCFG",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QCFG),

    DEF_CMD("+QFDEL",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QFDEL),
    DEF_CMD("+QFUPL",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QFUPL),

    DEF_CMD("+QGPS",      GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QGPS),
    DEF_CMD("+QGPSCFG",   GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QGPSCFG),
    DEF_CMD("+QGPSEND",   GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QGPSEND),
    DEF_CMD("+QGPSLOC",   GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QGPSLOC),

    DEF_CMD("+QIACT",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QIACT),
    DEF_CMD("+QICLOSE",   GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QICLOSE),
    DEF_CMD("+QICSGP",    GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QICSGP),
    DEF_CMD("+QIDEACT",   GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QIDEACT),
    DEF_CMD("+QIDNSCFG",   GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QIDNSCFG),
    DEF_CMD("+QIDNSGIP",   GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QIDNSGIP),
    DEF_CMD("+QIOPEN",    GS_RES_OK, GS_CMD_URC , GS_CMD_QIOPEN),
    DEF_CMD("+QIRD",      GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QIRD),
    DEF_CMD("+QISEND",    GS_RES_STR, GS_CMD_NORMAL , GS_CMD_QISEND),
    DEF_CMD("+QIURC",     GS_RES_OK, GS_CMD_URC , GS_CMD_QIURC),

    DEF_CMD("+QSSLCFG",    GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QSSLCFG),
    DEF_CMD("+QSSLCLOSE",  GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QSSLCLOSE),
    DEF_CMD("+QSSLOPEN",   GS_RES_OK, GS_CMD_URC , GS_CMD_QSSLOPEN),
    DEF_CMD("+QSSLRECV",   GS_RES_OK, GS_CMD_NORMAL , GS_CMD_QSSLRECV),
    DEF_CMD("+QSSLSEND",   GS_RES_STR, GS_CMD_NORMAL , GS_CMD_QSSLSEND),
    DEF_CMD("+QSSLURC",    GS_RES_OK, GS_CMD_URC , GS_CMD_QSSLURC),

};

extern GStatus gs;
extern GSOp gsops[MAX_OPS];
extern int gsopn;


void _gs_init(void);
void _gs_done(void);
int _gs_poweroff(void);
int _gs_poweron(void);
void _gs_loop(void *args);
int _gs_list_operators(void);
int _gs_set_operator(uint8_t* operator, int oplen);
int _gs_check_network(void);
int _gs_control_psd(int activate);
int _gs_config_psd();
int _gs_configure_psd(uint8_t *apn, int apnlen, uint8_t *username, int ulen, uint8_t* pwd, int pwdlen, int auth);
int _gs_set_rat(int rat, int band);

int _gs_get_rtc(uint8_t* time);
int _gs_rssi(void);
int _gs_attach(int attach);
int _gs_is_attached(void);
int _gs_imei(uint8_t  *imei);
int _gs_iccid(uint8_t *iccid);
int _gs_dns(uint8_t*dns);

int _gs_socket_connect(int id, NetAddress *addr);
int _gs_socket_new(int proto, int secure);
int _gs_socket_send(int id, uint8_t* buf, int len);
int _gs_socket_sendto(int id, uint8_t* buf, int len, NetAddress *addr);
int _gs_socket_recv(int id, uint8_t* buf, int len);
int _gs_socket_available(int id);
int _gs_socket_close(int id);
int _gs_resolve(uint8_t* url, int len, uint8_t* addr);
int _gs_socket_tls(int id, uint8_t* cacert, int cacertlen, uint8_t* clicert, int clicertlen, uint8_t* pvkey, int pvkeylen, int authmode);
int _gs_socket_bind(int id, NetAddress *addr);


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

int _gs_gnss_init(void);
int _gs_gnss_done(void);
int _gs_gnss_loc(GNSSLoc *loc);
