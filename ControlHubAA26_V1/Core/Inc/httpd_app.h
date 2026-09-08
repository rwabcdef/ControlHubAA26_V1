/**
  ******************************************************************************
  * @file    httpd_app.h
  * @brief   Application-side setup for the lwIP HTTP server.
  ******************************************************************************
  */

#ifndef HTTPD_APP_H
#define HTTPD_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Start the lwIP HTTP server.
  * @note   Must be called after MX_LWIP_Init(), from the lwIP thread context
  *         (the default task), since httpd uses the lwIP raw API.
  */
void HTTPD_App_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* HTTPD_APP_H */
