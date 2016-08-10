#include "mbedtls/platform.h"
#include "mbedtls/config.h"
#include "mbedtls/net.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/certs.h"
#include "mbedtls/x509.h"
#include "mbedtls/x509_crt.h"
//#include "mbedtls/debug.h"
#include <ctype.h>
#include <stdio.h>
#include <stdbool.h>
#include "mbedtls_Interface.h"
#include "mbedtls/pk.h"
#include "mbedtls/pk_internal.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/bignum.h"

#define DEBUG DEBUG_FULL
#include "net/ip/uip-debug.h"
#include "sys/process.h"

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
#include "mbedtls/memory_buffer_alloc.h"
#endif

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
    unsigned char alloc_buf[25000];
#endif


struct options opt = {
			DFL_SERVER_NAME,
                        DFL_SERVER_ADDR,
			DFL_SERVER_PORT,
			DFL_DEBUG_LEVEL,
			DFL_NBIO,
			DFL_READ_TIMEOUT,
			DFL_MAX_RESEND,
			DFL_REQUEST_PAGE,
			DFL_REQUEST_SIZE,
			DFL_CA_FILE,
			DFL_CA_PATH,
			DFL_CRT_FILE,
			DFL_KEY_FILE,
			DFL_PSK,
			DFL_PSK_IDENTITY,
			DFL_ECJPAKE_PW,
			{ DFL_FORCE_CIPHER_1,
			  DFL_FORCE_CIPHER_2
			},
			DFL_RENEGOTIATION,
			DFL_ALLOW_LEGACY,
			DFL_RENEGOTIATE,
			DFL_RENEGODELAY,
			DFL_EXCHANGES,
			DFL_MIN_VERSION,
			DFL_MAX_VERSION,
			DFL_ARC4,
			DFL_AUTH_MODE,//MBEDTLS_SSL_VERIFY_NONE,//MBEDTLS_SSL_VERIFY_OPTIONAL, //DFL_AUTH_MODE,
			DFL_MFL_CODE,
			DFL_TRUNC_HMAC,
			DFL_RECSPLIT,
			DFL_DHMLEN,
			DFL_RECONNECT,
			DFL_RECO_DELAY,
			DFL_RECONNECT_HARD,
			DFL_TICKETS,
			DFL_ALPN_STRING,
			DFL_TRANSPORT,
			DFL_HS_TO_MIN,
			DFL_HS_TO_MAX,
			DFL_FALLBACK,
			DFL_EXTENDED_MS,
			DFL_ETM
		     };


uip_ip6addr_t g_server_ip6addr = {DFL_SSL_SERVER_IPADDR};

/* Structure Variable declaration for mbedTLS client session */
static mbedtls_net_context server_fd;
static mbedtls_entropy_context entropy;
static mbedtls_ctr_drbg_context ctr_drbg;
static mbedtls_ssl_context ssl;
static mbedtls_ssl_config conf;
static mbedtls_ssl_session saved_session;

#if defined(MBEDTLS_TIMING_C)
 //   mbedtls_timing_delay_context timer;
#endif
#if defined(MBEDTLS_X509_CRT_PARSE_C)
static uint32_t flags;
static mbedtls_x509_crt cacert;
static mbedtls_x509_crt clicert;
static mbedtls_pk_context pkey;
#endif
//static char *q;
//static const int *list;

/* Variables required for ssl client */
int len, tail_len,  written, frags, retry_left;
//mbedtls_net_context server_fd;
//unsigned char buf[MBEDTLS_SSL_MAX_CONTENT_LEN + 1];
#if defined(MBEDTLS_KEY_EXCHANGE__SOME__PSK_ENABLED)
static unsigned char psk[MBEDTLS_PSK_MAX_LEN];
static   size_t psk_len = 0;
#endif
#if defined(MBEDTLS_SSL_ALPN)
    const char *alpn_list[10];
#endif
    const char *pers = "ssl_client2";


extern bool SSL_Handshake_Completed;

/*---------------------------------------------------------------------------*/
//PROCESS(mbedtls_client_process, "mbedtls client process");
/*---------------------------------------------------------------------------*/
#if 0
void mbedtls_client_event_handler(process_event_t ev, void *data)
{

    switch(ev) {
       
       case MBEDTLS_NODE_INITIAL_JOIN_DONE:
            PRINTF("mbedtls_client_app rcvd an event\n");
	    http_socket_init(&s);
            http_socket_post(&s, "http://127.0.0.1", NULL, NULL, //"http://172.16.2.86", NULL, NULL,   
                     NULL, NULL, NULL);
            break;

       case MBEDTLS_ESTABLISHED_CONN_TO_SERVER:
            break;

       case MBEDTLS_DATA_RECEIVED_FROM_SERVER:
            break;

       default:
            break;
    }

}
#endif
/*---------------------------------------------------------------------------*/
#if 0
PROCESS_THREAD(mbedtls_client_process, ev, data)
{
    PROCESS_BEGIN();
    
    while(1) {
       PROCESS_YIELD();
       mbedtls_client_event_handler(ev, data);
    }
    
    PROCESS_END();
}
#endif


static int mbedTLS_validate_cipher(void)
{
#if 1
	int ret = 0;
        const mbedtls_ssl_ciphersuite_t *ciphersuite_info;

        ciphersuite_info = mbedtls_ssl_ciphersuite_from_id( opt.force_ciphersuite[0] );

        if( opt.max_version != -1 &&
            ciphersuite_info->min_minor_ver > opt.max_version )
        {
            PRINTF("forced ciphersuite not allowed with this protocol version\n");
            ret = 2;
            return ret;
        }
        if( opt.min_version != -1 &&
            ciphersuite_info->max_minor_ver < opt.min_version )
        {
            PRINTF("forced ciphersuite not allowed with this protocol version\n");
            ret = 2;
            return ret;
        }

        /* If the server selects a version that's not supported by
         * this suite, then there will be no common ciphersuite... */
        if( opt.max_version == -1 ||
            opt.max_version > ciphersuite_info->max_minor_ver )
        {
            opt.max_version = ciphersuite_info->max_minor_ver;
        }
        if( opt.min_version < ciphersuite_info->min_minor_ver )
        {
            opt.min_version = ciphersuite_info->min_minor_ver;
            /* DTLS starts with TLS 1.1 */
            if( opt.transport == MBEDTLS_SSL_TRANSPORT_DATAGRAM &&
                opt.min_version < MBEDTLS_SSL_MINOR_VERSION_2 )
                opt.min_version = MBEDTLS_SSL_MINOR_VERSION_2;
        }

        /* Enable RC4 if needed and not explicitly disabled */
        if( ciphersuite_info->cipher == MBEDTLS_CIPHER_ARC4_128 )
        {
            if( opt.arc4 == MBEDTLS_SSL_ARC4_DISABLED )
            {
                PRINTF("forced RC4 ciphersuite with RC4 disabled\n");
                ret = 2;
                return ret;
            }

            opt.arc4 = MBEDTLS_SSL_ARC4_ENABLED;
        }
    return ret;
#endif //if 0
}
/*---------------------------------------------------------------------------*/

static int mbedTLS_unhexify_psk(void)
{
#if 1
   int ret = 0;
   unsigned char c;
   size_t j;
    /*
     * Unhexify the pre-shared key if any is given
     */
        if( strlen( opt.psk ) % 2 != 0 )
        {
            PRINTF("pre-shared key not valid hex\n");
            return 1;
        }

        psk_len = strlen( opt.psk ) / 2;

        for( j = 0; j < strlen( opt.psk ); j += 2 )
        {
            c = opt.psk[j];
            if( c >= '0' && c <= '9' )
                c -= '0';
            else if( c >= 'a' && c <= 'f' )
                c -= 'a' - 10;
            else if( c >= 'A' && c <= 'F' )
                c -= 'A' - 10;
            else
            {
                PRINTF("pre-shared key not valid hex\n");
                return 2;
            }
            psk[ j / 2 ] = c << 4;

            c = opt.psk[j + 1];
            if( c >= '0' && c <= '9' )
                c -= '0';
            else if( c >= 'a' && c <= 'f' )
                c -= 'a' - 10;
            else if( c >= 'A' && c <= 'F' )
                c -= 'A' - 10;
            else
            {
                PRINTF("pre-shared key not valid hex\n");
                return 3;
            }
            psk[ j / 2 ] |= c;
        }
    return ret;
#endif //if 0
}

#if defined(MBEDTLS_SSL_ALPN)
void mbedTLS_validate_ssl_alpn(void)
{
    char *p;
    int i;
        p = (char *) opt.alpn_string;
        i = 0;

        /* Leave room for a final NULL in alpn_list */
        while( i < (int) sizeof alpn_list - 1 && *p != '\0' )
        {
            alpn_list[i++] = p;

            /* Terminate the current string and move on to next one */
            while( *p != ',' && *p != '\0' )
                p++;
            if( *p == ',' )
                *p++ = '\0';
        }
}
#endif

/*---------------------------------------------------------------------------*/
int mbedtls_client_init(void)
{
    int i, ret = 0;
    //PRINTF("mbedtls_client_init started\n");
#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
    mbedtls_memory_buffer_alloc_init( alloc_buf, sizeof(alloc_buf) );
#endif
#if 1
  /*----- Step 1.a: Initialize mbedTLS structures and session data ------*/
    //mbedtls_net_init( &server_fd );
    //mbedtls_platform_set_printf(printf);
    mbedtls_ssl_init( &ssl );
    mbedtls_ssl_config_init( &conf );
    memset( &saved_session, 0, sizeof( mbedtls_ssl_session ) );
    mbedtls_ctr_drbg_init( &ctr_drbg );
#if defined(MBEDTLS_X509_CRT_PARSE_C)
    mbedtls_x509_crt_init( &cacert );
    mbedtls_x509_crt_init( &clicert );
    mbedtls_pk_init( &pkey );
#endif
#if defined(MBEDTLS_SSL_ALPN)
    memset( (void * ) alpn_list, 0, sizeof( alpn_list ) );
#endif

    /* Step 1.b: Validate the configured values. */
    if( opt.force_ciphersuite[0] > 0 ) {
         ret = mbedTLS_validate_cipher();
         if(ret) return ret;
    }
#if defined(MBEDTLS_KEY_EXCHANGE__SOME__PSK_ENABLED)
    /* Unhexify the pre-shared key if any is given */
    if( strlen( opt.psk ) ) {
         ret = mbedTLS_unhexify_psk();
	 if(ret) return ret;
    }
#endif /* MBEDTLS_KEY_EXCHANGE__SOME__PSK_ENABLED */
#if defined(MBEDTLS_SSL_ALPN)
    if( opt.alpn_string != NULL ) {
	mbedTLS_validate_ssl_alpn();
    }
#endif /* MBEDTLS_SSL_ALPN */

  /*------- Step 2.a: Initialize RNG and the session data --------------------*/
    /* @TBD currently for development enable MBEDTLS_TEST_NULL_ENTROPY macro. 
    Once network integration is done then add platform specific entropy source. */
    //PRINTF( "\n  . Seeding the random number generator..." );
    mbedtls_entropy_init( &entropy );
   if( ( ret = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy,
                      		(const unsigned char *) pers,
                               strlen( pers ) ) ) != 0 )
   {
        PRINTF( " failed\n  ! mbedtls_ctr_drbg_seed returned %d\n", ret );
        return ret;
   }
   //PRINTF( " ok\n" );

#if defined(MBEDTLS_X509_CRT_PARSE_C)
   /* Step 2.b: Load root certificates */
   //PRINTF( "  . Loading the CA root certificate ..." );
#if defined(MBEDTLS_CERTS_C)
   for( i = 0; mbedtls_test_cas[i] != NULL; i++ )
   {
       ret = mbedtls_x509_crt_parse( &cacert,
                                  (const unsigned char *) mbedtls_test_cas[i],
                                  mbedtls_test_cas_len[i] );
           PRINTF("@debug: mbedtls_test_cas[%d]\n", i);
            if( ret != 0 )
                break;
    }
#else
    {
        ret = 1;
        mbedtls_printf("MBEDTLS_CERTS_C not defined.");
    }
#endif
    if( ret < 0 )
    {
        PRINTF( " failed\n  !  mbedtls_x509_crt_parse returned -0x%x\n\n", -ret );
        return ret;
    }

    /* Step 2.c: Load own certificate and private key. */
    //PRINTF( "  . Loading the client cert. and key..." );
#if defined(MBEDTLS_CERTS_C)
        {
        ret = mbedtls_x509_crt_parse( &clicert, (const unsigned char *) mbedtls_test_cli_crt,
                mbedtls_test_cli_crt_len );
        }
#else
    {
        ret = 1;
        mbedtls_printf("MBEDTLS_CERTS_C not defined.");
    }
#endif
    if( ret != 0 )
    {
        PRINTF( " failed\n  !  mbedtls_x509_crt_parse returned -0x%x\n\n", -ret );
        return ret;
    }

#if defined(MBEDTLS_CERTS_C)
        ret = mbedtls_pk_parse_key( &pkey, (const unsigned char *) mbedtls_test_cli_key,
                mbedtls_test_cli_key_len, NULL, 0 );
#else
    {
        ret = 1;
        PRINTF("MBEDTLS_CERTS_C not defined.");
    }
#endif
    if( ret != 0 )
    {
        PRINTF( " failed\n  !  mbedtls_pk_parse_key returned -0x%x\n\n", -ret );
        return ret;
    }
    //PRINTF( " ok\n" );
#endif /* MBEDTLS_X509_CRT_PARSE_C */

#endif //if 0
    
    /* Step 3: Setup SSL/TLS structures */
    if( ( ret = mbedtls_ssl_config_defaults( &conf,
                    MBEDTLS_SSL_IS_CLIENT,
                    opt.transport,
                    MBEDTLS_SSL_PRESET_DEFAULT ) ) != 0 )
    {
        PRINTF( " failed\n  ! mbedtls_ssl_config_defaults returned -0x%x\n\n", -ret );
        return ret;
    }

    if( opt.auth_mode != DFL_AUTH_MODE )
        mbedtls_ssl_conf_authmode( &conf, opt.auth_mode );

#if defined(MBEDTLS_SSL_PROTO_DTLS)
    if( opt.hs_to_min != DFL_HS_TO_MIN || opt.hs_to_max != DFL_HS_TO_MAX )
        mbedtls_ssl_conf_handshake_timeout( &conf, opt.hs_to_min, opt.hs_to_max );
#endif /* MBEDTLS_SSL_PROTO_DTLS */

#if defined(MBEDTLS_SSL_MAX_FRAGMENT_LENGTH)
    if( ( ret = mbedtls_ssl_conf_max_frag_len( &conf, opt.mfl_code ) ) != 0 )
    {
        PRINTF( " failed\n  ! mbedtls_ssl_conf_max_frag_len returned %d\n\n", ret );
        return ret;
    }
#endif

#if defined(MBEDTLS_SSL_TRUNCATED_HMAC)
    if( opt.trunc_hmac != DFL_TRUNC_HMAC )
        mbedtls_ssl_conf_truncated_hmac( &conf, opt.trunc_hmac );
#endif

#if defined(MBEDTLS_SSL_EXTENDED_MASTER_SECRET)
    if( opt.extended_ms != DFL_EXTENDED_MS )
        mbedtls_ssl_conf_extended_master_secret( &conf, opt.extended_ms );
#endif

#if defined(MBEDTLS_SSL_ENCRYPT_THEN_MAC)
    if( opt.etm != DFL_ETM )
        mbedtls_ssl_conf_encrypt_then_mac( &conf, opt.etm );
#endif

#if defined(MBEDTLS_SSL_CBC_RECORD_SPLITTING)
    if( opt.recsplit != DFL_RECSPLIT )
        mbedtls_ssl_conf_cbc_record_splitting( &conf, opt.recsplit
                                    ? MBEDTLS_SSL_CBC_RECORD_SPLITTING_ENABLED
                                    : MBEDTLS_SSL_CBC_RECORD_SPLITTING_DISABLED );
#endif

#if defined(MBEDTLS_DHM_C)
    if( opt.dhmlen != DFL_DHMLEN )
        mbedtls_ssl_conf_dhm_min_bitlen( &conf, opt.dhmlen );
#endif

#if defined(MBEDTLS_SSL_ALPN)
    if( opt.alpn_string != NULL )
        if( ( ret = mbedtls_ssl_conf_alpn_protocols( &conf, alpn_list ) ) != 0 )
        {
            PRINTF( " failed\n  ! mbedtls_ssl_conf_alpn_protocols returned %d\n\n", ret );
            return ret;
        }
#endif
     mbedtls_ssl_conf_rng( &conf, mbedtls_ctr_drbg_random, &ctr_drbg );
     mbedtls_ssl_conf_read_timeout( &conf, opt.read_timeout );
#if defined(MBEDTLS_SSL_SESSION_TICKETS)
    mbedtls_ssl_conf_session_tickets( &conf, opt.tickets );
#endif

    if( opt.force_ciphersuite[0] != DFL_FORCE_CIPHER )
        mbedtls_ssl_conf_ciphersuites( &conf, opt.force_ciphersuite );

#if defined(MBEDTLS_ARC4_C)
    if( opt.arc4 != DFL_ARC4 )
        mbedtls_ssl_conf_arc4_support( &conf, opt.arc4 );
#endif

    if( opt.allow_legacy != DFL_ALLOW_LEGACY )
        mbedtls_ssl_conf_legacy_renegotiation( &conf, opt.allow_legacy );

#if defined(MBEDTLS_SSL_RENEGOTIATION)
    mbedtls_ssl_conf_renegotiation( &conf, opt.renegotiation );
#endif

#if defined(MBEDTLS_X509_CRT_PARSE_C)
    if( strcmp( opt.ca_path, "none" ) != 0 &&
        strcmp( opt.ca_file, "none" ) != 0 )
    {
        mbedtls_ssl_conf_ca_chain( &conf, &cacert, NULL );
    }
    if( strcmp( opt.crt_file, "none" ) != 0 &&
        strcmp( opt.key_file, "none" ) != 0 )
    {
        if( ( ret = mbedtls_ssl_conf_own_cert( &conf, &clicert, &pkey ) ) != 0 )
        {
            PRINTF( " failed\n  ! mbedtls_ssl_conf_own_cert returned %d\n\n", ret );
            return ret;
        }
    }
#endif

#if defined(MBEDTLS_KEY_EXCHANGE__SOME__PSK_ENABLED)
    if( ( ret = mbedtls_ssl_conf_psk( &conf, psk, psk_len,
                             (const unsigned char *) opt.psk_identity,
                             strlen( opt.psk_identity ) ) ) != 0 )
    {
        PRINTF( " failed\n  ! mbedtls_ssl_conf_psk returned %d\n\n", ret );
        return ret;
    }
    //PRINTF("psk:%s, psk_len:%d, psk_identity:%s\n", psk, psk_len, opt.psk_identity);
#endif

    if( opt.min_version != DFL_MIN_VERSION )
        mbedtls_ssl_conf_min_version( &conf, MBEDTLS_SSL_MAJOR_VERSION_3, opt.min_version );

    if( opt.max_version != DFL_MAX_VERSION )
        mbedtls_ssl_conf_max_version( &conf, MBEDTLS_SSL_MAJOR_VERSION_3, opt.max_version );

#if defined(MBEDTLS_SSL_FALLBACK_SCSV)
    if( opt.fallback != DFL_FALLBACK )
        mbedtls_ssl_conf_fallback( &conf, opt.fallback );
#endif

    if( ( ret = mbedtls_ssl_setup( &ssl, &conf ) ) != 0 )
    {
        PRINTF( " failed\n  ! mbedtls_ssl_setup returned -0x%x\n\n", -ret );
        return ret;
    }

#if defined(MBEDTLS_X509_CRT_PARSE_C)
    if( ( ret = mbedtls_ssl_set_hostname( &ssl, opt.server_name ) ) != 0 )
    {
        PRINTF( " failed\n  ! mbedtls_ssl_set_hostname returned %d\n\n", ret );
        return ret;
    }
#endif

#if defined(MBEDTLS_KEY_EXCHANGE_ECJPAKE_ENABLED)
    if( opt.ecjpake_pw != DFL_ECJPAKE_PW )
    {
        if( ( ret = mbedtls_ssl_set_hs_ecjpake_password( &ssl,
                        (const unsigned char *) opt.ecjpake_pw,
                                        strlen( opt.ecjpake_pw ) ) ) != 0 )
        {
            PRINTF( " failed\n  ! mbedtls_ssl_set_hs_ecjpake_password returned %d\n\n", ret );
            return ret;
        }
    }
#endif

    mbedtls_ssl_set_bio( &ssl, &server_fd, mbedtls_net_send, mbedtls_net_recv, NULL);

#if defined(MBEDTLS_TIMING_C)
    mbedtls_ssl_set_timer_cb( &ssl, &timer, mbedtls_timing_set_delay,
                                            mbedtls_timing_get_delay );
#endif

    PRINTF("mbedtls_client_init completed successfully\n");
    return ret;
}
/*---------------------------------------------------------------------------------*/
//extern unsigned int dbg_g_mbedtls_net_send;
extern unsigned int dbg_g_mbedtls_net_recv;
void my_debug_print(int ret)
{

    PRINTF("SSL handshake returned: State: %d,  ret: -0x%x,   net_recv:%d\n\n", ssl.state, -ret, dbg_g_mbedtls_net_recv);
    PRINTF("x509 certificate track: %d\n", dbg_g_get_ecdsa_verify());
}

/*---------------------------------------------------------------------------------*/

static uint8_t g_http_req[34] = {0x47, 0x45, 0x54, 0x20, 0x2f, 0x20, 0x48, 0x54, 0x54, 0x50, 0x2f, 0x31, 0x2e, 0x30, 0x0d, 0x0a, 0x45, 0x78, 0x74, 0x72, 0x61, 0x2d, 0x68, 0x65, 0x61, 0x64, 0x65, 0x72, 0x3a, 0x20, 0x0d, 0x0a, 0x0d, 0x0a};

void mbedtls_start_ssl_handshake(void)
{
    int ret = 0;
    //unsigned char buf[MBEDTLS_SSL_MAX_CONTENT_LEN + 1];
    
    PRINTF("mbedtls: SSL handshake started. state: %d. net_recv:%d\n", ssl.state, dbg_g_mbedtls_net_recv);
    //if(ssl.state)
    ret = mbedtls_ssl_handshake( &ssl );

    my_debug_print(ret);
    if(ret == 0) {
        SSL_Handshake_Completed = true;
    PRINTF("mbedtls: SSL handshake completed!!!\n");
    PRINTF("mbedtls: SSL handshake completed!!!\n");

    PRINTF("Send http request\n");
    ret = mbedtls_ssl_write(&ssl, g_http_req, sizeof(g_http_req));
    if(ret <= 0){
       PRINTF("ssl_write failed.\n");
    }
    else {
      PRINTF("Sent http request of %d bytes\n", ret);
    }
    //while(1);
#if 0
        PRINTF( " ok\n    [ Protocol is %s ]\n    [ Ciphersuite is %s ]\n",
            mbedtls_ssl_get_version( &ssl ), mbedtls_ssl_get_ciphersuite( &ssl ) );
#if defined(MBEDTLS_SSL_MAX_FRAGMENT_LENGTH)
        PRINTF( "    [ Maximum fragment length is %u ]\n",
                    (unsigned int) mbedtls_ssl_get_max_frag_len( &ssl ) );
#endif

#if defined(MBEDTLS_X509_CRT_PARSE_C)
    /* Verify the server certificate */
        PRINTF( "  . Verifying peer X.509 certificate..." );

        if( ( flags = mbedtls_ssl_get_verify_result( &ssl ) ) != 0 ) {
            char vrfy_buf[512];

            PRINTF( " failed\n" );
            mbedtls_x509_crt_verify_info( vrfy_buf, sizeof( vrfy_buf ), "  ! ", flags );
            PRINTF( "%s\n", vrfy_buf );
        }
        else
            PRINTF( " ok\n" );

       if( mbedtls_ssl_get_peer_cert( &ssl ) != NULL ) {
            PRINTF( "  . Peer certificate information    ...\n" );
            mbedtls_x509_crt_info( (char *) buf, sizeof( buf ) - 1, "      ",
                       mbedtls_ssl_get_peer_cert( &ssl ) );
            PRINTF( "%s\n", buf );
       }
#endif /* MBEDTLS_X509_CRT_PARSE_C */

#endif //0
    }
}


int mbedtls_ssl_data_rcvd(uint8_t *rcvd_data_ptr, int rcvd_data_len)
{
    int ret;
    ret = mbedtls_ssl_read( &ssl, rcvd_data_ptr, rcvd_data_len );
    return ret;
}

