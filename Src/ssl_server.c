/** 
  *
  *  Portions COPYRIGHT 2017 STMicroelectronics
  *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
  *
  ******************************************************************************
  * @file    mbedTLS/SSL_Server/Src/ssl_server.c
  * @author  MCD Application Team
  * @brief   SSL server application 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#include "main.h"
#include "cmsis_os.h"
#include <lwip.h>
#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#define mbedtls_time       time
#define mbedtls_time_t     time_t 
#define mbedtls_fprintf    fprintf
#define mbedtls_printf     printf
#endif

#include <stdlib.h>
#include <string.h>

#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/certs.h"
#include "mbedtls/x509.h"
#include "mbedtls/ssl.h"
#include "mbedtls/net_sockets.h"
#include "mbedtls/error.h"
#include "mbedtls/debug.h"

#if defined(MBEDTLS_SSL_CACHE_C)
#include "mbedtls/ssl_cache.h"
#endif



const char server_cert[] =
		"-----BEGIN CERTIFICATE-----\r\n"
		"MIIC/TCCAeWgAwIBAgIJAKrFcgGH6WauMA0GCSqGSIb3DQEBBQUAMBUxEzARBgNV\r\n"
		"BAMMCjEwLjEwLjkuODcwHhcNMTkwNTE2MjExMjA2WhcNMjkwNTEzMjExMjA2WjAV\r\n"
		"MRMwEQYDVQQDDAoxMC4xMC45Ljg3MIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIB\r\n"
		"CgKCAQEAsG6iGRnYLHHSzYICsEdkVwU1avbz5AykPtVhFdLPmOWBNuePCW9i77Bf\r\n"
		"toMBf+0/ClJ0XqR7BQeEmJs+BYhiNQAjjlYYWC5IhyL3f11fxt8Wfo1MCOR9nLrf\r\n"
		"XxjWriepIWjAr4gUNavoUn1GTnL/S2vUmO4lSyolKmlEh8wGDGeArT9pPraONHGp\r\n"
		"8Xbs+3tluVTlYVRi604cuiUR76rBI8Brvxydd/BonMDsnRJKLEZ4Ur8hjFNEKyEa\r\n"
		"WqEsNFDv/NyuZi31MNdTVutTEJmfNcouWPmVY3tqDF5uXDMKPPBASwU5fUG3fKrM\r\n"
		"qQh++5i6QjsLO9aaFNnngH+h4UusAwIDAQABo1AwTjAdBgNVHQ4EFgQUkeUf4Aoe\r\n"
		"O0Gbk3hvoM04usuojZcwHwYDVR0jBBgwFoAUkeUf4AoeO0Gbk3hvoM04usuojZcw\r\n"
		"DAYDVR0TBAUwAwEB/zANBgkqhkiG9w0BAQUFAAOCAQEAozV8Cok4O9rNWK2Mo173\r\n"
		"ny0ZEMRpvwhW3RF1VOrVQ2Cheo//9xD66LOxPP+5pIyGFeKQHZ0I2HPq8a5EYAMh\r\n"
		"SIt7prdbaVL0Evtn8ksmf5W7vPxUtMw7FhSVoFRUPd4c6Pia112X2JeIf1gm7tp9\r\n"
		"+eFK6zsL0P/0LH279XO10ZLuxxveYFiwk0tlm6PVkuliLoLCQC1UYysJWOhoxOTg\r\n"
		"G39MkVDMEMHXmo+GaYeFpkusezgAp0dq5HNmn/YFse9eKaS7/WGOFQ8tZK+mys2i\r\n"
		"f3t8chwndJpqhrwEql22VU1nnox1XdUBw3T7R+TajDz6rDwvyu6ou7fPi5oqduwo\r\n"
		"4g==\r\n"
		"-----END CERTIFICATE-----\r\n";

const char server_key[] =
		"-----BEGIN RSA PRIVATE KEY-----\r\n"
		"MIIEowIBAAKCAQEAsG6iGRnYLHHSzYICsEdkVwU1avbz5AykPtVhFdLPmOWBNueP\r\n"
		"CW9i77BftoMBf+0/ClJ0XqR7BQeEmJs+BYhiNQAjjlYYWC5IhyL3f11fxt8Wfo1M\r\n"
		"COR9nLrfXxjWriepIWjAr4gUNavoUn1GTnL/S2vUmO4lSyolKmlEh8wGDGeArT9p\r\n"
		"PraONHGp8Xbs+3tluVTlYVRi604cuiUR76rBI8Brvxydd/BonMDsnRJKLEZ4Ur8h\r\n"
		"jFNEKyEaWqEsNFDv/NyuZi31MNdTVutTEJmfNcouWPmVY3tqDF5uXDMKPPBASwU5\r\n"
		"fUG3fKrMqQh++5i6QjsLO9aaFNnngH+h4UusAwIDAQABAoIBAQCFGlHMIgVGMnDY\r\n"
		"fXbNym5WgCV2luwUZb0bchsWPb5EGRDMfREaCyBWt3bd2COvgSqH/QCxzeDJQIy2\r\n"
		"HJhD0EpWtc4dObKQBArPeH/DZIw3GmJ8AtkT9F0Xepu9wJ1lMaKVA9QqGL02aISS\r\n"
		"htQdpwAWxDjVTwRvmlIDlvsOf8soW0N8FFcO6AmctB6f794EjDxD0FTB866RBdcT\r\n"
		"doNu/mNFQZZRnMb5LhXVlBLynnXxU0L66XeNUKRj9THF6wozxEDm9GEa57KK2JQJ\r\n"
		"SslBlnhvB3gYOBwMcyKW8h4usbiLdRYDbXwNrEJLaG5otTZQmzc2cc8+GuEXvTlC\r\n"
		"e2eOoo+hAoGBAOgem1UTH2SrDl8Trr3as0kzGW2s7YbSzB5FsaAptgo5GFOhm9v0\r\n"
		"oM/MQ7y3tMTzyIT8Mro8ZibiLvAhveFJH0DLEUMjM9Z8QbS/Y5GnLWaPVxk1sq/p\r\n"
		"uINQZ5SjsU4ez9fPHwAaCP0ID/mIioT9tlI28COkBnuSEqm22KUM75DJAoGBAMKV\r\n"
		"X3pFDJTTrhrDsZlNMgYipmq88V+GY3xnJWvCXQul0ZdWaVbEmOi65eurfbkqUcBb\r\n"
		"Ue8B7VVYzW9kQ1xSf7XI5XvQQ6W37xD67dvLvWtP7eTPWrcu76xj+szqw0oeE6R4\r\n"
		"m90Py6c/ZOTUPFPJKqh9EwRsYY6TWx0bZ/P8VOhrAoGAWvFq8IkeeqWeatfeVdoy\r\n"
		"9lID+3mOUo2SlaW+sz7EaPr3sgSWeTY+L0wbmfvr4mKVRK0+/sKdT0y9ES5XI8Yv\r\n"
		"bYZiTAilzbo/UB7QVwfF/PHaMTFdwhhzR1egTZdY53+g9S/cOID61pBrGdxKUVLK\r\n"
		"NSn7KiugspeHomDWPsxzrCkCgYBa3wNdyI1dm5W9bhKssz5fWyM4ydA3ej1PxaPW\r\n"
		"NUjava+p63L6UInQdigV7VqjdL4FBSC4a7/4kaYvFTXYEbcOoCl7rwFIjSOaXHVk\r\n"
		"pjtpuEQzDRsiXUsUyQRBWomlPXKS5rhzfoLvMk3eB7e8sT+4u+B7ulm9CdgNOdVM\r\n"
		"PlUwzQKBgB6+OpjsspIIPlk1mYZp287UNzlZlDvUGu5lqNYzHd32JzuTt5FTzm3+\r\n"
		"YNdPeKAbKXWlhPAlaG4lCb4W29BngsK6TtTcejMENY6RuUty8WSRFfEF8kIfHtCP\r\n"
		"Gdn5cyqZTxs4GrRFdPjQAmb4ZLeol4OEnU09Tgu4X8f8HZK2lxxF\r\n"
		"-----END RSA PRIVATE KEY-----\r\n";



static mbedtls_net_context listen_fd, client_fd;
static uint8_t buf[1024];
static const char *pers = "ssl_server";
mbedtls_entropy_context entropy;
mbedtls_ctr_drbg_context ctr_drbg;
mbedtls_ssl_context ssl;
mbedtls_ssl_config conf;
mbedtls_x509_crt srvcert;
mbedtls_pk_context pkey;

#if defined(MBEDTLS_SSL_CACHE_C)
  mbedtls_ssl_cache_context cache;
#endif
  /**
   * Debug callback for mbed TLS
   * Just prints on the USB serial port
   */
  static void my_debug(void *ctx, int level, const char *file, int line,
                       const char *str)
  {
      const char *p, *basename;
      (void) ctx;

      /* Extract basename from file */
      for(p = basename = file; *p != '\0'; p++) {
          if(*p == '/' || *p == '\\') {
              basename = p + 1;
          }
      }

      mbedtls_printf("%s:%04d: |%d| %s", basename, line, level, str);
  }


void SSL_Server(void const *argument)
{
  int ret, len;
  UNUSED(argument);
  
  
#ifdef MBEDTLS_MEMORY_BUFFER_ALLOC_C
  mbedtls_memory_buffer_alloc_init(memory_buf, sizeof(memory_buf));
#endif
  mbedtls_net_init( &listen_fd );
  mbedtls_net_init( &client_fd );
  mbedtls_ssl_init( &ssl );
  mbedtls_ssl_config_init( &conf );
  mbedtls_ssl_conf_dbg(&conf, my_debug, NULL);
  mbedtls_debug_set_threshold(0);
#if defined(MBEDTLS_SSL_CACHE_C)
  mbedtls_ssl_cache_init( &cache );
#endif
  mbedtls_x509_crt_init( &srvcert );
  mbedtls_pk_init( &pkey );
  mbedtls_entropy_init( &entropy );
  mbedtls_ctr_drbg_init( &ctr_drbg );

  /*
   * 1. Load the certificates and private RSA key
   */
  mbedtls_printf( "\n  . Loading the server cert. and key..." );

  /*
   * This demonstration program uses embedded test certificates.
   * Using mbedtls_x509_crt_parse_file() to read the server and CA certificates
   * resuires the implmentation of the File I/O API using the FatFs, that is 
   * not implemented yet.
   */
#if defined(MBEDTLS_FS_IO)
  ret = mbedtls_x509_crt_parse_file(&srvcert, "servercert.cert");
#else
  ret = mbedtls_x509_crt_parse( &srvcert, (const unsigned char *) server_cert, sizeof(server_cert) );
#endif
  //ret = mbedtls_x509_crt_parse( &srvcert, (const unsigned char *) mbedtls_test_srv_crt, mbedtls_test_srv_crt_len );

  if(ret != 0)
  {
    mbedtls_printf( " failed\n  !  mbedtls_x509_crt_parse returned %d\n\n", ret );
    goto exit;
  }
/*
  ret = mbedtls_x509_crt_parse(&srvcert, (const unsigned char *) mbedtls_test_cas_pem, mbedtls_test_cas_pem_len);
  if( ret != 0 )
  {
    mbedtls_printf(" failed\n  !  mbedtls_x509_crt_parse returned %d\n\n", ret);
    goto exit;
  }
*/
#if defined(MBEDTLS_FS_IO)
  ret = mbedtls_pk_parse_keyfile(&pkey, "serverkey.key", NULL);
#else
  ret =  mbedtls_pk_parse_key(&pkey, (const unsigned char *) server_key, sizeof(server_key), NULL, 0);
#endif
  //ret =  mbedtls_pk_parse_key( &pkey, (const unsigned char *) mbedtls_test_srv_key, mbedtls_test_srv_key_len, NULL, 0 );
  if( ret != 0 )
  {
    mbedtls_printf(" failed\n  !  mbedtls_pk_parse_key returned %d\n\n", ret);
    goto exit;
  }

  mbedtls_printf( " ok\n" );

  /*
   * 2. Setup the listening TCP socket
   */

  mbedtls_printf( "  . Bind on https://%s:%s/ ...", get_current_ipaddress(),SERVER_PORT );

  if((ret = mbedtls_net_bind(&listen_fd, get_current_ipaddress(), SERVER_PORT , MBEDTLS_NET_PROTO_TCP )) != 0)
  {
    mbedtls_printf( " failed\n  ! mbedtls_net_bind returned %d\n\n", ret );
    goto exit;
  }

  mbedtls_printf( " ok\n" );

  /*
   * 3. Seed the RNG
   */
  mbedtls_printf( "  . Seeding the random number generator..." );

  if((ret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy, (const unsigned char *) pers, strlen( (char *)pers))) != 0)
  {
    mbedtls_printf( " failed\n  ! mbedtls_ctr_drbg_seed returned %d\n", ret );
    goto exit;
  }

  mbedtls_printf( " ok\n" );

  /*
   * 4. Setup stuff
   */
  mbedtls_printf( "  . Setting up the SSL data...." );

  if((ret = mbedtls_ssl_config_defaults(&conf, MBEDTLS_SSL_IS_SERVER, MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT)) != 0)
  {
    mbedtls_printf( " failed\n  ! mbedtls_ssl_config_defaults returned %d\n\n", ret );
    goto exit;
  }

  mbedtls_ssl_conf_rng(&conf, mbedtls_ctr_drbg_random, &ctr_drbg);

#if defined(MBEDTLS_SSL_CACHE_C)
  mbedtls_ssl_conf_session_cache(&conf, &cache, mbedtls_ssl_cache_get, mbedtls_ssl_cache_set);
#endif

  mbedtls_ssl_conf_ca_chain(&conf, srvcert.next, NULL);
  if((ret = mbedtls_ssl_conf_own_cert(&conf, &srvcert, &pkey)) != 0)
  {
    mbedtls_printf( " failed\n  ! mbedtls_ssl_conf_own_cert returned %d\n\n", ret );
    goto exit;
  }

  if((ret = mbedtls_ssl_setup(&ssl, &conf)) != 0)
  {
    mbedtls_printf( " failed\n  ! mbedtls_ssl_setup returned %d\n\n", ret );
    goto exit;
  }

  mbedtls_printf( " ok\n" );

reset:
#ifdef MBEDTLS_ERROR_C
  if(ret != 0)
  {
    uint8_t error_buf[100];
    mbedtls_strerror( ret, (char *)error_buf, 100 );
    mbedtls_printf("Last error was: %d - %s\n\n", ret, error_buf );
  }
#endif
  
  mbedtls_net_free(&client_fd);

  mbedtls_ssl_session_reset(&ssl);

  /*
   * 5. Wait until a client connects
   */
  mbedtls_printf( "  . Waiting for a remote connection ...\n" );

  if((ret = mbedtls_net_accept(&listen_fd, &client_fd, NULL, 0, NULL)) != 0)
  {
    mbedtls_printf(" failed\n  ! mbedtls_net_accept returned %d\n\n", ret);
    goto exit;
  }

  mbedtls_ssl_set_bio(&ssl, &client_fd, mbedtls_net_send, mbedtls_net_recv, NULL);

  mbedtls_printf(" ok\n");

  /*
   * 6. Handshake
   */
  mbedtls_printf("  . Performing the SSL/TLS handshake...");

  while((ret = mbedtls_ssl_handshake(&ssl)) != 0)
  {
    if(ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE)
    {
      mbedtls_printf(" failed\n  ! mbedtls_ssl_handshake returned -0x%x\n\n", -ret);
      goto reset;
    }
  }

  mbedtls_printf(" ok\n");

  /*
   * 7. Read the HTTP Request
   */
  mbedtls_printf("  < Read from client:");
  do
  {
    len = sizeof(buf) - 1;
    memset(buf, 0, sizeof(buf));
    BSP_LED_Off(LED_GREEN);
    BSP_LED_Off(LED_RED);
    ret = mbedtls_ssl_read(&ssl, buf, len);

    if(ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE)
    {
      continue;
    }
    if(ret <= 0)
    {
      switch(ret)
      {
        case MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY:
          mbedtls_printf(" connection was closed gracefully\n");
          HAL_Delay(100);
          BSP_LED_On(LED_RED);
          break;

        case MBEDTLS_ERR_NET_CONN_RESET:
          mbedtls_printf(" connection was reset by peer\n");
          HAL_Delay(100);
          BSP_LED_On(LED_RED);
          break;

        default:
          mbedtls_printf(" mbedtls_ssl_read returned -0x%x\n", -ret);
          HAL_Delay(100);
          BSP_LED_On(LED_RED);
          break;
      }
      
      HAL_Delay(100);
      BSP_LED_On(LED_GREEN);
      break;
    }

    len = ret;
    mbedtls_printf(" %d bytes read\n\n%s", len, (char *) buf);

    if(ret > 0)
    {
      break;
    }
  } while(1);

  /*
   * 8. Write the 200 Response
   */
  mbedtls_printf( "  > Write to client:" );
  len = sprintf((char *) buf, HTTP_RESPONSE, mbedtls_ssl_get_ciphersuite(&ssl));

  while((ret = mbedtls_ssl_write(&ssl, buf, len)) <= 0)
  {
    if(ret == MBEDTLS_ERR_NET_CONN_RESET)
    {
      mbedtls_printf(" failed\n  ! peer closed the connection\n\n");
      goto reset;
    }

    if(ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE)
    {
      mbedtls_printf( " failed\n  ! mbedtls_ssl_write returned %d\n\n", ret );
      goto exit;
    }
  }

  len = ret;
  mbedtls_printf(" %d bytes written\n\n%s\n", len, (char *) buf);

  mbedtls_printf("  . Closing the connection...");

  while((ret = mbedtls_ssl_close_notify(&ssl)) < 0)
  {
    if(ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE)
    {
      mbedtls_printf( " failed\n  ! mbedtls_ssl_close_notify returned %d\n\n", ret );
      goto reset;
    }
  }

  if (ret == 0)
  {
    BSP_LED_On(LED_GREEN);
    mbedtls_printf( " ok\n" );
  }
  
  ret = 0;
  goto reset;

exit:
  BSP_LED_Off(LED_GREEN);
  BSP_LED_On(LED_RED);

  mbedtls_net_free( &client_fd );
  mbedtls_net_free( &listen_fd );

  mbedtls_x509_crt_free( &srvcert );
  mbedtls_pk_free( &pkey );
  mbedtls_ssl_free( &ssl );
  mbedtls_ssl_config_free( &conf );
#if defined(MBEDTLS_SSL_CACHE_C)
  mbedtls_ssl_cache_free( &cache );
#endif
  mbedtls_ctr_drbg_free( &ctr_drbg );
  mbedtls_entropy_free( &entropy );

}