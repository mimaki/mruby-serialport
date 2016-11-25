/*
 * SerialPort class for mruby
 *
 * Copyright (c) 2014 Monami-ya LLC, Japan. and others
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#include <string.h>
#include <unistd.h>
// #include <fcntl.h>
// #include <termios.h>
// #include <sys/ioctl.h>
#include "serialport-impl.h"
#include "mruby/data.h"
#include "mruby/error.h"

#define termios _DCB

mrb_int mrb_serial_create_port_impl(mrb_state *mrb, mrb_value self)
{
  mrb_value val;
  enum mrb_vtype vtype;
  mrb_int fileno;
  HANDLE hComm;

  mrb_get_args(mrb, "o", &val);

  vtype = mrb_type(val);

  if (vtype == MRB_TT_STRING) {
    struct RString *rstring;
//     struct termios params;
    char path[PATH_MAX];
    mrb_int len;
    COMMTIMEOUTS cto;

    rstring = mrb_str_ptr(val);
    len = mrb_str_strlen(mrb, rstring);
    if (len < 0 || (size_t)len > PATH_MAX) {
      mrb_raise(mrb, E_ARGUMENT_ERROR, "Invalid pathname.");
    }
    memcpy(path, RSTRING_PTR(val), (size_t)len + 1);

    // fileno = open(path, O_RDWR | O_NOCTTY | O_NDELAY);
    hComm = CreateFile(path, GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    // if (fileno == -1) {
    if (hComm == INVALID_HANDLE_VALUE) {
      mrb_raise(mrb, E_IO_ERROR, "Can't open serialport.");
    }
    fileno = (mrb_int)hComm;

    // if (!isatty(fileno)) {
    //   close(fileno);
    //   mrb_raise(mrb, E_ARGUMENT_ERROR, "not a serial port");
    // }

    /* blocking read */
//     {
//       int fl;
//       fl = fcntl(fileno, F_GETFL, 0);
//       fcntl(fileno, F_SETFL, fl & ~O_NONBLOCK);
//     }
    if (!GetCommTimeouts(hComm, &cto)) {
      CloseHandle(hComm);
      mrb_raise(mrb, E_ARGUMENT_ERROR, "not a serial port");
    }
    cto.ReadIntervalTimeout = 50; /* 50ms */
    SetCommTimeouts(hComm, &cto);

//     /* Don't use serial_tcgetattr()
//      * because this instance has not been initialized yet. */
//     if (tcgetattr(fileno, &params) == -1) {
//       close(fileno);
//       mrb_raise(mrb, E_IO_ERROR, "tcgetattr");
//     }

//     params.c_oflag = 0;
//     params.c_lflag = 0;
//     params.c_iflag &= (IXON | IXOFF | IXANY);
//     params.c_cflag |= CLOCAL | CREAD;
//     params.c_cflag &= ~HUPCL;

//     if (tcsetattr(fileno, TCSANOW, &params) == -1) {
//       close(fileno);
//       mrb_raise(mrb, E_IO_ERROR, "tcsetattr");
//     }
  }
  else {
    if (vtype == MRB_TT_FIXNUM) {
      mrb_raise(mrb, E_ARGUMENT_ERROR, "Illegal port number");
    }
    else {
      mrb_raise(mrb, E_TYPE_ERROR, "Wring argument type");
    }
    fileno = -1;
  }

  return fileno;
}


static int
get_fd_helper(mrb_state *mrb, mrb_value self)
{
  return mrb_fixnum(mrb_funcall(mrb, self, "fileno", 0));
}

struct termios;

static inline void
serial_tcgetattr(mrb_state *mrb, mrb_value self, struct termios *params)
{
  int fd;

  fd = get_fd_helper(mrb, self);
//   if (tcgetattr(fd, params) == -1) {
//     mrb_raise(mrb, E_IO_ERROR, "tcgetattr");
//   }
  params->DCBlength = sizeof(*params);
  if (!GetCommState((HANDLE)fd, params)) {
    mrb_raise(mrb, E_IO_ERROR, "GetCommState failed.");
  }
}

static inline void
serial_tcsetattr(mrb_state *mrb, mrb_value self, struct termios *params)
{
  int fd;

  fd = get_fd_helper(mrb, self);
//   if (tcsetattr(fd, TCSANOW, params) == -1) {
//     mrb_raise(mrb, E_IO_ERROR, "tcsetattr");
//   }
  params->DCBlength = sizeof(*params);
  if (!SetCommState((HANDLE)fd, params)) {
    mrb_raise(mrb, E_IO_ERROR, "SetCommState failed.");
  }
}

static inline int
serial_convert_baud_rate(mrb_state *mrb, struct modem_params_t *modem_params)
{
//   int baud_rate;

//   switch(modem_params->baud_rate) {
//   case     50: baud_rate =     B50; break;
//   case     75: baud_rate =     B75; break;
//   case    110: baud_rate =    B110; break;
//   case    134: baud_rate =    B134; break;
//   case    150: baud_rate =    B150; break;
//   case    200: baud_rate =    B200; break;
//   case    300: baud_rate =    B300; break;
//   case    600: baud_rate =    B600; break;
//   case   1200: baud_rate =   B1200; break;
//   case   1800: baud_rate =   B1800; break;
//   case   2400: baud_rate =   B2400; break;
//   case   4800: baud_rate =   B4800; break;
//   case   9600: baud_rate =   B9600; break;
//   case  19200: baud_rate =  B19200; break;
//   case  38400: baud_rate =  B38400; break;
// #ifdef B57600
//   case  57600: baud_rate =  B57600; break;
// #endif
// #ifdef B76800
//   case  76800: baud_rate =  B76800; break;
// #endif
// #ifdef B115200
//   case 115200: baud_rate = B115200; break;
// #endif
// #ifdef B230400
//   case 230400: baud_rate = B230400; break;
// #endif
//   default:
//     mrb_raise(mrb, E_ARGUMENT_ERROR, "unknown baud rate");
//     baud_rate = 0; /* to supress a warning. */
//     break;
//   }

//   return baud_rate;
    return modem_params->baud_rate;
}

void
mrb_serial_break_impl(mrb_state *mrb, mrb_value self, mrb_int time)
{
  mrb_raise(mrb, E_IO_ERROR, "mrb_serial_break_impl");
//   int fd;
//   fd = get_fd_helper(mrb, self);

//   if (tcsendbreak(fd, time * 4 / 10) == -1) {   /* 0.4sec * time */
//     mrb_raise(mrb, E_IO_ERROR, "tcsendbreak");
//   }
}

mrb_int
mrb_serial_flow_control_impl(mrb_state *mrb, mrb_value self)
{
//   struct termios params;
  mrb_int result = 0;
  mrb_raise(mrb, E_IO_ERROR, "mrb_serial_flow_control_impl");

//   serial_tcgetattr(mrb, self, &params);

//   if (params.c_cflag & CRTSCTS) {
//     result += MRBGEM_SERIALPORT_HARD;
//   }

//   if (params.c_iflag & (IXON | IXOFF | IXANY)) {
//     result += MRBGEM_SERIALPORT_SOFT;
//   }

  return result;
}

void
mrb_serial_set_flow_control_impl(mrb_state *mrb, mrb_value self, mrb_int flow_control)
{
  mrb_raise(mrb, E_IO_ERROR, "mrb_serial_set_flow_control_impl");
//   struct termios params;

//   serial_tcgetattr(mrb, self, &params);

//   if (flow_control & MRBGEM_SERIALPORT_HARD) {
//     params.c_cflag |= CRTSCTS;
//   }
//   else {
//     params.c_cflag &= ~CRTSCTS;
//   }

//   if (flow_control & MRBGEM_SERIALPORT_SOFT) {
//     params.c_iflag |= (IXON | IXOFF | IXANY);
//   }
//   else {
//     params.c_iflag &= ~(IXON | IXOFF | IXANY);
//   }

//   serial_tcsetattr(mrb, self, &params);
}

mrb_value
mrb_read_timeout_impl(mrb_state *mrb, mrb_value self)
{
  mrb_raise(mrb, E_IO_ERROR, "mrb_read_timeout_impl");
//   mrb_value result;
//   struct termios params;

//   serial_tcgetattr(mrb, self, &params);

//   if (params.c_cc[VTIME] == 0 && params.c_cc[VMIN] == 0) {
//     result = mrb_fixnum_value(-1);
//   }

//   if (MRB_INT_MAX / params.c_cc[VTIME] <= 100) {
//     /* convert to float beacuse there is no Bignum on mruby */
//     result = mrb_float_value(mrb, params.c_cc[VTIME] * 100.0);
//   }
//   else {
//     result = mrb_fixnum_value(params.c_cc[VTIME] * 100);
//   }

//   return result;
    return mrb_true_value();
}

void
mrb_set_read_timeout_impl(mrb_state *mrb, mrb_value self, mrb_int timeout)
{
  mrb_raise(mrb, E_IO_ERROR, "mrb_set_read_timeout_impl");
//   struct termios params;

//   serial_tcgetattr(mrb, self, &params);

//   if (timeout < 0) {
//     params.c_cc[VTIME] = 0;
//     params.c_cc[VMIN] = 0;
//   }
//   else if (timeout == 0) {
//     params.c_cc[VTIME] = 0;
//     params.c_cc[VMIN] = 1;
//   }
//   else {
//     params.c_cc[VTIME] = (timeout + 50) / 100;
//     params.c_cc[VMIN] = 0;
//   }

//   serial_tcgetattr(mrb, self, &params);
}

mrb_int
mrb_write_timeout_impl(mrb_state *mrb, mrb_value self)
{
  mrb_raise(mrb, E_IO_ERROR, "mrb_write_timeout_impl");
//   int fd;
//   fd = get_fd_helper(mrb, self);

//   mrb_raise(mrb, E_NOTIMP_ERROR, "not implemented");

  return 0; /* NOT REACHED */
}

void
mrb_set_write_timeout_impl(mrb_state *mrb, mrb_value self, mrb_int time)
{
  mrb_raise(mrb, E_IO_ERROR, "mrb_set_write_timeout_impl");
//   int fd;
//   fd = get_fd_helper(mrb, self);

//   mrb_raise(mrb, E_NOTIMP_ERROR, "not implemented");
}

void
mrb_serial_get_modem_params_impl(mrb_state *mrb, mrb_value self, struct modem_params_t *modem_params)
{
  struct termios params;

  serial_tcgetattr(mrb, self, &params);

//   switch (cfgetospeed(&params)) {
//   case B50:     modem_params->baud_rate = 50;    break;
//   case B75:     modem_params->baud_rate = 75;    break;
//   case B110:    modem_params->baud_rate = 110;   break;
//   case B134:    modem_params->baud_rate = 134;   break;
//   case B150:    modem_params->baud_rate = 150;   break;
//   case B200:    modem_params->baud_rate = 200;   break;
//   case B300:    modem_params->baud_rate = 300;   break;
//   case B600:    modem_params->baud_rate = 600;   break;
//   case B1200:   modem_params->baud_rate = 1200;  break;
//   case B1800:   modem_params->baud_rate = 1800;  break;
//   case B2400:   modem_params->baud_rate = 2400;  break;
//   case B4800:   modem_params->baud_rate = 4800;  break;
//   case B9600:   modem_params->baud_rate = 9600;  break;
//   case B19200:  modem_params->baud_rate = 19200; break;
//   case B38400:  modem_params->baud_rate = 38400; break;
// #ifdef B57600
//   case B57600:  modem_params->baud_rate = 57600; break;
// #endif
// #ifdef B76800
//   case B76800:  modem_params->baud_rate = 76800; break;
// #endif
// #ifdef B115200
//   case B115200: modem_params->baud_rate = 115200; break;
// #endif
// #ifdef B230400
//   case B230400: modem_params->baud_rate = 230400; break;
// #endif
//   }
  modem_params->baud_rate = params.BaudRate;

//   switch(params.c_cflag & CSIZE) {
//   case CS5:
//     modem_params->data_bits = 5;
//     break;
//   case CS6:
//     modem_params->data_bits = 6;
//     break;
//   case CS7:
//     modem_params->data_bits = 7;
//     break;
//   case CS8:
//     modem_params->data_bits = 8;
//     break;
//   default:
//     modem_params->data_bits = 0;
//     break;
//   }
  modem_params->data_bits = params.ByteSize;

//   modem_params->stop_bits = (params.c_cflag & CSTOPB ? 2 : 1);
  switch (params.ByteSize) {
  case ONESTOPBIT:
  default:
    modem_params->stop_bits = 1;
    break;
  case TWOSTOPBITS:
    modem_params->stop_bits = 2;
    break;
  }  

//   if (!(params.c_cflag & PARENB)) {
//     modem_params->parity = MRBGEM_SERIALPORT_NONE;
//   }
//   else if (params.c_cflag & PARODD) {
//     modem_params->parity = MRBGEM_SERIALPORT_ODD;
//   }
//   else {
//     modem_params->parity = MRBGEM_SERIALPORT_EVEN;
//   }
  modem_params->parity = params.Parity;
}

void
mrb_serial_get_signals_impl(mrb_state *mrb, mrb_value self, struct line_signals_t *signals)
{
  mrb_raise(mrb, E_IO_ERROR, "mrb_serial_get_signals_impl");
//   int fd;
//   int line_status;

//   fd = get_fd_helper(mrb, self);

//   if (ioctl(fd, TIOCMGET, &line_status) == -1) {
//     mrb_raise(mrb, E_IO_ERROR, "TIOCMGET");
//   }

//   signals->rts = (line_status & TIOCM_RTS ? 1 : 0);
//   signals->dtr = (line_status & TIOCM_DTR ? 1 : 0);
//   signals->cts = (line_status & TIOCM_CTS ? 1 : 0);
//   signals->dsr = (line_status & TIOCM_DSR ? 1 : 0);
//   signals->dcd = (line_status & TIOCM_CD  ? 1 : 0);
//   signals->ri  = (line_status & TIOCM_RI  ? 1 : 0);
}

void
mrb_serial_set_modem_params_impl(mrb_state *mrb, mrb_value self, struct modem_params_t *modem_params)
{
  struct termios params;
  // int baud_rate;
//   int data_bits[] = { CS5, CS6, CS7, CS8 };

  serial_tcgetattr(mrb, self, &params);

//   baud_rate = serial_convert_baud_rate(mrb, modem_params);
//   cfsetispeed(&params, baud_rate);
//   cfsetospeed(&params, baud_rate);
  params.BaudRate = modem_params->baud_rate;

  if (modem_params->data_bits < 5 ||
      modem_params->data_bits > 8) {
    mrb_raise(mrb, E_ARGUMENT_ERROR, "unknown character size");
  }

//   params.c_cflag &= ~CSIZE;
//   params.c_cflag |= data_bits[modem_params->data_bits - 5];
  params.ByteSize = modem_params->data_bits;

  switch(modem_params->stop_bits) {
  case 1:
//     params.c_cflag &= ~CSTOPB;
    params.StopBits = ONESTOPBIT;
    break;
  case 2:
//     params.c_cflag |= CSTOPB;
    params.StopBits = TWOSTOPBITS;
    break;
  default:
    mrb_raise(mrb, E_ARGUMENT_ERROR, "unknown number of stop bits");
    break;
  }

  switch(modem_params->parity) {
  case MRBGEM_SERIALPORT_EVEN:
  //   params.c_cflag |= PARENB;
  //   params.c_cflag &= ~PARODD;
    params.Parity = EVENPARITY;
    break;
  case MRBGEM_SERIALPORT_ODD:
  //   params.c_cflag |= PARENB;
  //   params.c_cflag |= PARODD;
    params.Parity = ODDPARITY;
    break;
  case MRBGEM_SERIALPORT_NONE:
  //   params.c_cflag &= ~PARENB;
    params.Parity = NOPARITY;
    break;
  default:
    mrb_raise(mrb, E_ARGUMENT_ERROR, "unknown parity");
    break;
  }
  params.Parity = modem_params->parity;

  serial_tcsetattr(mrb, self, &params);
}


void
mrb_serial_set_signals_impl(mrb_state *mrb, mrb_value self, struct line_signals_t *signals)
{
  mrb_raise(mrb, E_IO_ERROR, "mrb_serial_set_signals_impl");
//   int fd, line_status;

//   fd = get_fd_helper(mrb, self);

//   if (ioctl(fd, TIOCMGET, &line_status) == -1) {
//     mrb_raise(mrb, E_IO_ERROR, "TIOCMGET");
//   }

//   /* set: 1 = on, -1 = off, 0 = keep previous */
//   if (signals->rts) {
//     line_status &= (signals->rts == 1) ? TIOCM_RTS : ~TIOCM_RTS;
//   }
//   if (signals->dtr) {
//     line_status &= (signals->dtr == 1) ? TIOCM_DTR : ~TIOCM_DTR;
//   }
//   if (signals->cts) {
//     line_status &= (signals->cts == 1) ? TIOCM_CTS : ~TIOCM_CTS;
//   }
//   if (signals->dsr) {
//     line_status &= (signals->dsr == 1) ? TIOCM_DSR : ~TIOCM_DSR;
//   }
//   if (signals->dcd) {
//     line_status &= (signals->dcd == 1) ? TIOCM_CD  : ~TIOCM_CD;
//   }
//   if (signals->ri) {
//     line_status &= (signals->ri  == 1) ? TIOCM_RI  : ~TIOCM_RI;
//   }

//  if (ioctl(fd, TIOCMSET, &line_status) == -1) {
//    mrb_raise(mrb, E_IO_ERROR, "TIOCMSET");
//  }
}

extern struct mrb_data_type mrb_io_type;

// #define _USE_WINAPI

mrb_value
mrb_serial_sysread_impl(mrb_state *mrb, mrb_value self)
{
  struct mrb_io *fptr;
  mrb_value buf = mrb_nil_value();
  mrb_int maxlen;
// #ifndef _USE_WINAPI
//   int ret;
// #else
  DWORD ret;
// #endif

  mrb_get_args(mrb, "i|S", &maxlen, &buf);
  if (maxlen < 0) {
    return mrb_nil_value();
  }

  if (mrb_nil_p(buf)) {
    buf = mrb_str_new(mrb, NULL, maxlen);
  }
  if (RSTRING_LEN(buf) != maxlen) {
    buf = mrb_str_resize(mrb, buf, maxlen);
  }

  fptr = (struct mrb_io *)mrb_get_datatype(mrb, self, &mrb_io_type);
// #ifndef _USE_WINAPI /* POSIX */
//   ret = read(fptr->fd, RSTRING_PTR(buf), maxlen);
// #else /* WinAPI */
  if (!ReadFile((HANDLE)fptr->fd, RSTRING_PTR(buf), maxlen, &ret, NULL)) {
    mrb_sys_fail(mrb, "sysread failed");
  }
// #endif
  switch (ret) {
    case 0: /* EOF */
      if (maxlen == 0) {
        buf = mrb_str_new_cstr(mrb, "");
      } else {
        mrb_raise(mrb, E_EOF_ERROR, "sysread failed: End of File");
      }
      break;
// #ifndef _USE_WINAPI
//     case -1: /* Error */
//       mrb_sys_fail(mrb, "sysread failed");
//       break;
// #endif
    default:
      if (RSTRING_LEN(buf) != ret) {
        buf = mrb_str_resize(mrb, buf, ret);
      }
      break;
  }

  return buf;
}

mrb_value
mrb_serial_syswrite_impl(mrb_state *mrb, mrb_value self)
{
  struct mrb_io *fptr;
  mrb_value str, buf;
// #ifndef _USE_WINAPI /* POSIX */
//   int fd, length;
// #else /* WinAPI */
  int fd;
  DWORD length;
// #endif

  fptr = (struct mrb_io *)mrb_get_datatype(mrb, self, &mrb_io_type);
  if (! fptr->writable) {
    mrb_raise(mrb, E_IO_ERROR, "not opened for writing");
  }

  mrb_get_args(mrb, "S", &str);
  if (mrb_type(str) != MRB_TT_STRING) {
    buf = mrb_funcall(mrb, str, "to_s", 0);
  } else {
    buf = str;
  }

  if (fptr->fd2 == -1) {
    fd = fptr->fd;
  } else {
    fd = fptr->fd2;
  }
// #ifndef _USE_WINAPI /* POSIX */
//   length = write(fd, RSTRING_PTR(buf), RSTRING_LEN(buf));
// #else /* WinAPI */
  WriteFile((HANDLE)fd, RSTRING_PTR(buf), RSTRING_LEN(buf), &length, NULL);
// #endif

  return mrb_fixnum_value(length);
}

#endif /* _WIN32, _WIN64 */
