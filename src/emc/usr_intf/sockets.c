/********************************************************************
* Description: sockets.c
*   socket utilities
*
* Copyright(c) 2001, Joris Robijn
*          (c) 2003, Rene Wagner
* Adapted for EMC by: Eric H. Johnson
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2007 All rights reserved.
*
* Last change:
********************************************************************/

#include "config.h"
#include <unistd.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>

#ifndef WINSOCK2
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#else
#include <winsock2.h>
#endif

#include <stdarg.h>
#include <fcntl.h>

#include "rcs_print.hh"
#include "sockets.h"

/**************************************************
*  LCDproc client sockets code...
*  Note: LCDproc error reporting was replaced
*        with EMC standard debug reporting.
**************************************************/

// Length of longest transmission allowed at once...
#define MAXMSG 8192

typedef struct sockaddr_in sockaddr_in;

static int sockInitSockaddr(sockaddr_in *name, const char *hostname, unsigned short int port)
{
  struct hostent *hostinfo;

  memset(name, '\0', sizeof (*name));
  name->sin_family = AF_INET;
  name->sin_port = htons(port);
  hostinfo = gethostbyname(hostname);
  if (hostinfo == NULL) {
    rcs_print_error("sock_init_sockaddr: Unknown host\n");
    return -1;
    }
  name->sin_addr = *(struct in_addr *) hostinfo->h_addr;

  return 0;
}

// Client functions...
int sockConnect(char *host, unsigned short int port)
{
  struct sockaddr_in servername;
  int sock;
  int err = 0;

  rcs_print_error("sock_connect: Creating socket\n");
  sock = socket(PF_INET, SOCK_STREAM, 0);
#ifdef WINSOCK2        
  if (sock == INVALID_SOCKET) {
#else
  if (sock < 0) {
#endif
    rcs_print_error("sock_connect: Error creating socket\n");
    return sock;
    }
  rcs_print_error("sock_connect: Created socket\n");

  if (sockInitSockaddr(&servername, host, port) < 0)
    return -1;

  err = connect(sock, (struct sockaddr *) &servername, sizeof (servername));
#ifdef WINSOCK2        
  if (err == INVALID_SOCKET) {
#else
  if (err < 0) {
#endif
    rcs_print_error("sock_connect: connect failed\n");
    shutdown(sock, SHUT_RDWR);
    return -1;
    }

#ifndef WINSOCK2        
  fcntl(sock, F_SETFL, O_NONBLOCK);
#else
  {
    unsigned long tmp = 1;
    if (ioctlsocket(sock, FIONBIO, &tmp) == SOCKET_ERROR)
      rcs_print_error("sock_connect: Error setting socket to non-blocking\n");
  }
#endif

  return sock;
}

int sockClose(int fd)
{
  int err;

  err = shutdown(fd, SHUT_RDWR);
  if (!err) close (fd);

  return err;
}


/**  send printf-like formatted output */
int sockPrintf(int fd, const char *format, .../*args*/ )
{
  char buf[MAXMSG];
  va_list ap;
  int size = 0;

  va_start(ap, format);
  size = vsnprintf(buf, sizeof(buf), format, ap);
  va_end(ap);

  if (size < 0) {
    rcs_print_error("sock_printf: vsnprintf failed\n");
    return -1;
    }
  if (size > (int)sizeof(buf)) {
    rcs_print_error("sock_printf: vsnprintf truncated message\n");
    }
  return sockSendString(fd, buf);
}

// Send/receive lines of text
int sockSendString(int fd, const char *string)
{
  return sockSend(fd, string, strlen(string));
}

// Recv gives only one line per call...
int sockRecvString(int fd, char *dest, size_t maxlen)
{
  char *ptr = dest;
  int recvBytes = 0;

  if (!dest) return -1;
  if (maxlen <= 0) return 0;

  while (1) {
    int err = recv(fd, ptr, 1, 0);
    if (err == -1) {
      if (errno == EAGAIN) {
        if (recvBytes) {
          // We've begun to read a string, but no bytes are
          // available.  Loop.
          continue;
          }
        return 0;
        } 
      else {
        rcs_print_error("sock_recv_string: socket read error");
        return err;
        }
      } 
    else if (err == 0) {
      return recvBytes;
      }

    recvBytes++;

    // stop at max. bytes allowed, at NUL or at LF
    if (recvBytes == (int)maxlen || *ptr == '\0' || *ptr == '\n') {
      *ptr = '\0';
      break;
      }
    ptr++;
    }

  // Don't return an empty string
  if (recvBytes == 1 && dest[0] == '\0')
    return 0;

  if (recvBytes < (int)maxlen - 1)
    dest[recvBytes] = '\0';

  return recvBytes;
}

// Send/receive raw data
int sockSend(int fd, const void *src, size_t size)
{
  size_t offset = 0;

  if (!src) return -1;

  while (offset != size) {
    // write isn't guaranteed to send the entire string at once,
    // so we have to sent it in a loop like this
#ifndef WINSOCK2
    ssize_t sent = write(fd, ((const char *) src) + offset, size - offset);
#else
    ssize_t sent = send(fd, ((const char *) src) + offset, size - offset, 0);
#endif
    if (sent == -1) {
      if (errno != EAGAIN) {
        rcs_print_error("sock_send: socket write error\n");
//      shutdown(fd, SHUT_RDWR);
        return sent;
        }
      continue;
      } 
    else if (sent == 0) return sent + offset;
    offset += sent;
    } // while

  return (int)offset;
}

int sockRecv(int fd, void *dest, size_t maxlen)
{
  int err;

  if (!dest) return -1;
  if (maxlen <= 0) return 0;

#ifndef WINSOCK2
  err = read (fd, dest, maxlen);
#else
  err = recv(fd, dest, maxlen, 0);
#endif
  if (err < 0) {
//  rcs_print_error("sock_recv: socket read error\n");
//  shutdown(fd, SHUT_RDWR);
    return err;
    }

  return err;
}

/*****************************************************************************/

char* sockGetError(void)
{
#ifndef WINSOCK2
  return strerror(errno);
#else
  static char retString[256];
  long err;
  char* tmp = NULL;

  err = WSAGetLastError();

  snprintf(retString, sizeof(retString), "Error code %ld: ", err);
  FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | 
                FORMAT_MESSAGE_FROM_SYSTEM | 
                FORMAT_MESSAGE_IGNORE_INSERTS,
                NULL,
                err,
                0, /* Default language */
                (LPTSTR) &tmp,
                0,
                NULL);

    /* append the message text after the error code and ensure a terminating
       character ends the string */
  // The 'tmp' buffer is allocated by FormatMessage().
  // cppcheck-suppress nullPointer
  strncpy(retString + strlen(retString), tmp, 
          sizeof(retString) - strlen(retString) - 1);
  retString[sizeof(retString) - 1] = '\0';
  LocalFree(tmp);

  return retString;
#endif
}

/** prints error to logfile and sends it to the client.
 * @param fd socket
 * @param message the message to send (without the "huh? ") */
int sockSendError(int fd, const char* message)
{
// simple: performance penalty isn't worth more work...
  return sockPrintfError(fd, "%s", message);
}

/** prints printf-like formatted output to logfile and sends it to the
 * client.
 * @note don't add a the "huh? " to the message. This is done by this
 *   method
 * @param fd socket
 * @param format a printf format */
int sockPrintfError(int fd, const char *format, .../*args*/ )
{
  static const char huh[] = "huh? ";
  char buf[MAXMSG];
  va_list ap;
  int size = 0;

  strncpy(buf, huh, sizeof(buf));

  va_start(ap, format);
  size = vsnprintf(buf + (sizeof(huh)-1), sizeof(buf) - (sizeof(huh)-1), format, ap);
  buf[sizeof(buf)-1] = '\0';
  va_end(ap);

  if (size < 0) {
    rcs_print_error("sock_printf_error: vsnprintf failed\n");
    return -1;
    }
  if (size >= (int)(sizeof(buf) - (sizeof(huh)-1))) {
    rcs_print_error("sock_printf_error: vsnprintf truncated message\n");
    }

  return sockSendString(fd, buf);
}
