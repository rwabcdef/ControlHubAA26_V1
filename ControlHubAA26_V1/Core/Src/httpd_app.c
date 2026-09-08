/**
  ******************************************************************************
  * @file    httpd_app.c
  * @brief   Application-side setup for the lwIP HTTP server.
  *
  * Web content served by httpd lives in Core/Inc/fsdata_custom.h.
  ******************************************************************************
  */

#include "httpd_app.h"

#include "lwip/apps/httpd.h"
#include "lwip/apps/fs.h"
#include "lwip/debug.h"

void HTTPD_App_Init(void)
{
  httpd_init();
}

/**
  * @brief  Generic CGI handler, called once for every URI that carries query
  *         parameters (e.g. "/index.html?led=on").
  * @note   lwipopts.h sets LWIP_HTTPD_CGI_SSI to 1, which makes this symbol
  *         mandatory: httpd.c calls it unconditionally, so omitting it is an
  *         undefined-reference at link time. The stub below is a valid no-op;
  *         add parameter handling here as the application grows.
  */
void httpd_cgi_handler(struct fs_file *file, const char *uri, int iNumParams,
                       char **pcParam, char **pcValue)
{
  LWIP_UNUSED_ARG(file);
  LWIP_UNUSED_ARG(uri);
  LWIP_UNUSED_ARG(iNumParams);
  LWIP_UNUSED_ARG(pcParam);
  LWIP_UNUSED_ARG(pcValue);
}
