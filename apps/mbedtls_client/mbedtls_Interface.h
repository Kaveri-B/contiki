/*
*  Header file which contains default settings for mbedTLS client application.
*/
#ifndef MBEDTLS_INTERFACE_H_
#define MBEDTLS_INTERFACE_H_

#define DFL_SERVER_NAME         "localhost"
#define DFL_SERVER_ADDR         NULL
#define DFL_SERVER_PORT         "4433"
#define DFL_REQUEST_PAGE        "/"
#define DFL_REQUEST_SIZE        -1
#define DFL_DEBUG_LEVEL         0
#define DFL_NBIO                1 //Non-blocking I/O
#define DFL_READ_TIMEOUT        0
#define DFL_MAX_RESEND          0
#define DFL_CA_FILE             ""
#define DFL_CA_PATH             ""
#define DFL_CRT_FILE            ""
#define DFL_KEY_FILE            ""
#define DFL_PSK                 ""
#define DFL_PSK_IDENTITY        "Client_identity"
#define DFL_ECJPAKE_PW          NULL
#define DFL_FORCE_CIPHER_1      MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_CCM_8
#define DFL_FORCE_CIPHER_2	0
#define DFL_FORCE_CIPHER	0
#define DFL_RENEGOTIATION       MBEDTLS_SSL_RENEGOTIATION_DISABLED
#define DFL_ALLOW_LEGACY        -2
#define DFL_RENEGOTIATE         0
#define DFL_RENEGODELAY         0
#define DFL_EXCHANGES           1
#define DFL_MIN_VERSION         -1
#define DFL_MAX_VERSION         -1
#define DFL_ARC4                -1
#define DFL_AUTH_MODE           -1
#define DFL_MFL_CODE            MBEDTLS_SSL_MAX_FRAG_LEN_NONE
#define DFL_TRUNC_HMAC          -1
#define DFL_RECSPLIT            -1
#define DFL_DHMLEN              -1
#define DFL_RECONNECT           0
#define DFL_RECO_DELAY          0
#define DFL_RECONNECT_HARD      0
#define DFL_TICKETS             MBEDTLS_SSL_SESSION_TICKETS_ENABLED
#define DFL_ALPN_STRING         NULL
#define DFL_TRANSPORT           MBEDTLS_SSL_TRANSPORT_STREAM
#define DFL_HS_TO_MIN           0
#define DFL_HS_TO_MAX           0
#define DFL_FALLBACK            -1
#define DFL_EXTENDED_MS         -1
#define DFL_ETM                 -1

#define GET_REQUEST "GET %s HTTP/1.0\r\nExtra-header: "
#define GET_REQUEST_END "\r\n\r\n"

#define DFL_SSL_SERVER_IPADDR {0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}

/* mbedtls events */
#define MBEDTLS_NODE_INITIAL_JOIN_DONE		0x01
#define MBEDTLS_ESTABLISHED_CONN_TO_SERVER	0x02
#define MBEDTLS_DATA_RECEIVED_FROM_SERVER	0x03

/*
 * global options
 */
struct options
{
    const char *server_name;    /* hostname of the server (client only)     */
    const char *server_addr;    /* address of the server (client only)      */
    const char *server_port;    /* port on which the ssl service runs       */
    int debug_level;            /* level of debugging                       */
    int nbio;                   /* should I/O be blocking?                  */
    uint32_t read_timeout;      /* timeout on mbedtls_ssl_read() in milliseconds    */
    int max_resend;             /* DTLS times to resend on read timeout     */
    const char *request_page;   /* page on server to request                */
    int request_size;           /* pad request with header to requested size */
    const char *ca_file;        /* the file with the CA certificate(s)      */
    const char *ca_path;        /* the path with the CA certificate(s) reside */
    const char *crt_file;       /* the file with the client certificate     */
    const char *key_file;       /* the file with the client key             */
    const char *psk;            /* the pre-shared key                       */
    const char *psk_identity;   /* the pre-shared key identity              */
    const char *ecjpake_pw;     /* the EC J-PAKE password                   */
    int force_ciphersuite[2];   /* protocol/ciphersuite to use, or all      */
    int renegotiation;          /* enable / disable renegotiation           */
    int allow_legacy;           /* allow legacy renegotiation               */
    int renegotiate;            /* attempt renegotiation?                   */
    int renego_delay;           /* delay before enforcing renegotiation     */
    int exchanges;              /* number of data exchanges                 */
    int min_version;            /* minimum protocol version accepted        */
    int max_version;            /* maximum protocol version accepted        */
    int arc4;                   /* flag for arc4 suites support             */
    int auth_mode;              /* verify mode for connection               */
    unsigned char mfl_code;     /* code for maximum fragment length         */
    int trunc_hmac;             /* negotiate truncated hmac or not          */
    int recsplit;               /* enable record splitting?                 */
    int dhmlen;                 /* minimum DHM params len in bits           */
    int reconnect;              /* attempt to resume session                */
    int reco_delay;             /* delay in seconds before resuming session */
    int reconnect_hard;         /* unexpectedly reconnect from the same port */
    int tickets;                /* enable / disable session tickets         */
    const char *alpn_string;    /* ALPN supported protocols                 */
    int transport;              /* TLS or DTLS?                             */
    uint32_t hs_to_min;         /* Initial value of DTLS handshake timer    */
    uint32_t hs_to_max;         /* Max value of DTLS handshake timer        */
    int fallback;               /* is this a fallback connection?           */
    int extended_ms;            /* negotiate extended master secret?        */
    int etm;                    /* negotiate encrypt then mac?              */
};

extern bool SSL_Handshake_Completed;

/* APIs provided by mbedtls_client_app. */
int mbedtls_client_init(void);
int mbedtls_handle_rcvd_data(uint8_t *rcvd_data_ptr, int rcvd_data_len);
void mbedtls_start_ssl_handshake(void);

#endif /* MBEDTLS_INTERFACE_H_ */
