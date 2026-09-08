/**
  ******************************************************************************
  * @file    fsdata_custom.h
  * @brief   lwIP httpd read-only filesystem image (web content).
  *
  * This file replaces the stock "fsdata.c" that lwIP's fs.c expects to #include.
  * STM32CubeMX enables LWIP_HTTPD but does not generate this file, so it must be
  * maintained here. It is selected via HTTPD_FSDATA_FILE in lwipopts.h.
  *
  * It is a .h (not .c) deliberately: fs.c textually includes it, and a .c file
  * anywhere under a CubeIDE source folder would ALSO be compiled as its own
  * translation unit, producing duplicate symbols at link time.
  *
  * Content notes:
  *  - LWIP_HTTPD_DYNAMIC_HEADERS is 0, so each file must carry its own HTTP
  *    response header, and be marked FS_FILE_FLAGS_HEADER_INCLUDED.
  *  - LWIP_HTTPD_SUPPORT_11_KEEPALIVE is 0, so responses use "Connection: close"
  *    and the flag FS_FILE_FLAGS_HEADER_PERSISTENT is deliberately NOT set.
  *  - Files are a singly-linked list; FS_ROOT points at the head. Add new files
  *    by defining them above the current head and chaining "next" through them.
  *  - To serve real content later, generate this from HTML with lwIP's
  *    makefsdata tool (lwip-contrib/apps/) and keep the same include wiring.
  ******************************************************************************
  */

#ifndef FSDATA_CUSTOM_H
#define FSDATA_CUSTOM_H

#include "lwip/apps/fs.h"

/* ---------------------------------------------------------------- /404.html */

static const char fsname_404_html[] = "/404.html";

static const unsigned char fsdata_404_html[] =
  "HTTP/1.1 404 Not Found\r\n"
  "Content-Type: text/html\r\n"
  "Connection: close\r\n"
  "\r\n"
  "<!DOCTYPE html>\n"
  "<html lang=\"en\">\n"
  "<head><meta charset=\"utf-8\"><title>404 Not Found</title></head>\n"
  "<body>\n"
  "<h1>404 &ndash; Not Found</h1>\n"
  "<p>The requested resource does not exist on this device.</p>\n"
  "</body>\n"
  "</html>\n";

/* -------------------------------------------------------------- /index.html */

static const char fsname_index_html[] = "/index.html";

static const unsigned char fsdata_index_html[] =
  "HTTP/1.1 200 OK\r\n"
  "Content-Type: text/html\r\n"
  "Connection: close\r\n"
  "\r\n"
  "<!DOCTYPE html>\n"
  "<html lang=\"en\">\n"
  "<head><meta charset=\"utf-8\"><title>ControlHub AA26</title></head>\n"
  "<body>\n"
  "<h1>ControlHub AA26</h1>\n"
  "<p>STM32F439ZI &middot; lwIP httpd is running.</p>\n"
  "</body>\n"
  "</html>\n";

/* ------------------------------------------------------------- file table -- */
/* sizeof() - 1 drops the compiler-added terminating NUL, which is not content. */

const struct fsdata_file file_404_html[] = { {
    NULL,
    (const unsigned char *)fsname_404_html,
    fsdata_404_html,
    sizeof(fsdata_404_html) - 1,
    FS_FILE_FLAGS_HEADER_INCLUDED
  }
};

const struct fsdata_file file_index_html[] = { {
    file_404_html,
    (const unsigned char *)fsname_index_html,
    fsdata_index_html,
    sizeof(fsdata_index_html) - 1,
    FS_FILE_FLAGS_HEADER_INCLUDED
  }
};

#define FS_ROOT file_index_html

#endif /* FSDATA_CUSTOM_H */
