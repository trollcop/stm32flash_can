/*
  stm32flash - Open Source ST STM32 flash program for *nix
  Copyright (C) 2010 Geoffrey McRae <geoff@spacevs.com>
  Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#define _CRT_SECURE_NO_WARNINGS

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include <windows.h>
#include <ctype.h>

#include "compiler.h"
#include "serial.h"
#include "port.h"
#include "utils.h"

#define SLCAN_BUF_SIZE (22 * 2)

struct serial {
	HANDLE fd;
	DCB oldtio;
	DCB newtio;
	char setup_str[11];
	char slcan_buf[SLCAN_BUF_SIZE];
	uint16_t can_id;
};

static serial_t *slcan_open(const char *device)
{
	serial_t *h = calloc(sizeof(serial_t), 1);
	char devName[MAX_PATH] = { 0, };
	char *comPtr = NULL;

	/* timeout in ms */
	COMMTIMEOUTS timeouts = {MAXDWORD, MAXDWORD, 500, 0, 0};

	if (!h)
		return NULL;

	/* Fix the device name if required */
	if (strlen(device) > 4 && device[0] != '\\') {
		snprintf(devName, MAX_PATH, "\\\\.\\%s", device);
	} else {
		snprintf(devName, MAX_PATH, device);
	}
	/* Replace CAN with COM */
	if ((comPtr = strstr(devName, "CAN"))) {
		comPtr[1] = 'O';
		comPtr[2] = 'M';
	}

	/* Create file handle for port */
	h->fd = CreateFile(devName, GENERIC_READ | GENERIC_WRITE,
			   0,		/* Exclusive access */
			   NULL,	/* No security */
			   OPEN_EXISTING,
			   0,		/* No overlap */
			   NULL);

	if (h->fd == INVALID_HANDLE_VALUE) {
		if (GetLastError() == ERROR_FILE_NOT_FOUND)
			fprintf(stderr, "File not found: %s\n", device);
		free(h);
		return NULL;
	}

	SetupComm(h->fd, 4096, 4096); /* Set input and output buffer size */

	SetCommTimeouts(h->fd, &timeouts);

	SetCommMask(h->fd, EV_ERR); /* Notify us of error events */

	/* DCBlength should be initialized before calling GetCommState */
	h->oldtio.DCBlength = sizeof(DCB);
	h->newtio.DCBlength = sizeof(DCB);
	GetCommState(h->fd, &h->oldtio); /* Retrieve port parameters */
	GetCommState(h->fd, &h->newtio); /* Retrieve port parameters */

	/* PurgeComm(h->fd, PURGE_RXABORT | PURGE_TXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR); */

	return h;
}

static void slcan_flush(const serial_t __unused *h)
{
	/* We shouldn't need to flush in non-overlapping (blocking) mode */
	/* tcflush(h->fd, TCIFLUSH); */
}

static void slcan_close(serial_t *h)
{
	slcan_flush(h);
	SetCommState(h->fd, &h->oldtio);
	CloseHandle(h->fd);
	free(h);
}

static port_err_t slcan_setup(serial_t *h, const serial_baud_t baud)
{
	switch (baud) {
		case SERIAL_BAUD_1200:    h->newtio.BaudRate = CBR_1200; break;
		/* case SERIAL_BAUD_1800: h->newtio.BaudRate = CBR_1800; break; */
		case SERIAL_BAUD_2400:    h->newtio.BaudRate = CBR_2400; break;
		case SERIAL_BAUD_4800:    h->newtio.BaudRate = CBR_4800; break;
		case SERIAL_BAUD_9600:    h->newtio.BaudRate = CBR_9600; break;
		case SERIAL_BAUD_19200:   h->newtio.BaudRate = CBR_19200; break;
		case SERIAL_BAUD_38400:   h->newtio.BaudRate = CBR_38400; break;
		case SERIAL_BAUD_57600:   h->newtio.BaudRate = CBR_57600; break;
		case SERIAL_BAUD_115200:  h->newtio.BaudRate = CBR_115200; break;
		case SERIAL_BAUD_128000:  h->newtio.BaudRate = CBR_128000; break;
		case SERIAL_BAUD_256000:  h->newtio.BaudRate = CBR_256000; break;
		/* These are not defined in WinBase.h and might work or not */
		case SERIAL_BAUD_230400:  h->newtio.BaudRate = 230400; break;
		case SERIAL_BAUD_460800:  h->newtio.BaudRate = 460800; break;
		case SERIAL_BAUD_500000:  h->newtio.BaudRate = 500000; break;
		case SERIAL_BAUD_576000:  h->newtio.BaudRate = 576000; break;
		case SERIAL_BAUD_921600:  h->newtio.BaudRate = 921600; break;
		case SERIAL_BAUD_1000000: h->newtio.BaudRate = 1000000; break;
		case SERIAL_BAUD_1500000: h->newtio.BaudRate = 1500000; break;
		case SERIAL_BAUD_2000000: h->newtio.BaudRate = 2000000; break;
		case SERIAL_BAUD_INVALID:

		default:
			return PORT_ERR_UNKNOWN;
	}

	/* slcan is fixed at 8N1 */
	h->newtio.ByteSize = 8;
	h->newtio.Parity = NOPARITY;
	h->newtio.StopBits = ONESTOPBIT;

	/* reset the settings */
	h->newtio.fOutxCtsFlow = FALSE;
	h->newtio.fOutxDsrFlow = FALSE;
	h->newtio.fDtrControl = DTR_CONTROL_DISABLE;
	h->newtio.fDsrSensitivity = FALSE;
	h->newtio.fTXContinueOnXoff = FALSE;
	h->newtio.fOutX = FALSE;
	h->newtio.fInX = FALSE;
	h->newtio.fErrorChar = FALSE;
	h->newtio.fNull = FALSE;
	h->newtio.fRtsControl = RTS_CONTROL_DISABLE;
	h->newtio.fAbortOnError = FALSE;

	/* set the settings */
	slcan_flush(h);
	if (!SetCommState(h->fd, &h->newtio))
		return PORT_ERR_UNKNOWN;

	snprintf(h->setup_str, sizeof(h->setup_str), "%u 8N1", serial_get_baud_int(baud));
	return PORT_ERR_OK;
}

static port_err_t slcan_w32_open(struct port_interface *port, struct port_options *ops)
{
	serial_t *h;
	DWORD r;

	/* 1. check device name match */
	if (!(!strncmp(ops->device, "CAN", 3) && isdigit(ops->device[3]))
	    && !(!strncmp(ops->device, "\\\\.\\CAN", strlen("\\\\.\\CAN"))
		 && isdigit(ops->device[strlen("\\\\.\\CAN")])))
		return PORT_ERR_NODEV;

	/* 2. check options
	   SLCAN is fixed at 8N1 and any baudrate (beacuse USB->Serial) */
	if (ops->baudRate == SERIAL_BAUD_INVALID)
		return PORT_ERR_UNKNOWN;

	/* 3. open it */
	h = slcan_open(ops->device);
	if (h == NULL)
		return PORT_ERR_UNKNOWN;

	/* 4. set options */
	if (slcan_setup(h, ops->baudRate) != PORT_ERR_OK) {
		slcan_close(h);
		return PORT_ERR_UNKNOWN;
	}

	/* Open slcan interface */
	sprintf(h->slcan_buf, "O\r");
	WriteFile(h->fd, h->slcan_buf, 2, &r, NULL);

	port->private = h;
	return PORT_ERR_OK;
}

static port_err_t slcan_w32_close(struct port_interface *port)
{
	serial_t *h;
	DWORD r;

	h = (serial_t *)port->private;
	if (h == NULL)
		return PORT_ERR_UNKNOWN;

	/* Close slcan interface */
	sprintf(h->slcan_buf, "C\r");
	WriteFile(h->fd, h->slcan_buf, 2, &r, NULL);

	slcan_close(h);
	port->private = NULL;
	return PORT_ERR_OK;
}

static port_err_t slcan_packet_parse(struct port_interface *port, uint8_t *buf, size_t bufsiz, size_t *nbytes)
{
	serial_t *h;
	char *ptr;
	uint32_t tmp;
	uint16_t can_id = 0;
	uint8_t dlc = 0;
	uint8_t i;

	h = (serial_t *)port->private;
	if (h == NULL)
		return PORT_ERR_UNKNOWN;

	ptr = h->slcan_buf;
	if (ptr[0] != 't') {
		*nbytes = 0;
		return PORT_ERR_UNKNOWN;
	}
	ptr++;
	if (sscanf(ptr, "%03x", &tmp) != 1) {
		*nbytes = 0;
		return PORT_ERR_UNKNOWN;
	}
	can_id = tmp & 0x7FF;
	ptr += 3;
	dlc = ptr[0] - '0';
	ptr++;

	if (dlc > 8)
		dlc = 8;

	for (i = 0; i < dlc; i++) {
		uint8_t chr;
		if (sscanf(ptr, "%02x", &tmp) != 1) {
			*nbytes = i;
			return PORT_ERR_UNKNOWN;
		}
		chr = tmp & 0xff;
		ptr += 2;
		if (i < bufsiz)
			buf[i] = chr;
	}

	h->can_id = can_id;
	*nbytes = i;

	return PORT_ERR_OK;
}

static port_err_t slcan_w32_read(struct port_interface *port, void *buf, size_t nbyte)
{
	serial_t *h;
	DWORD r;
	size_t rd = 0, bytes_read = 0;
	uint8_t *ptr = (uint8_t *)buf;

	h = (serial_t *)port->private;
	if (h == NULL)
		return PORT_ERR_UNKNOWN;

	memset(h->slcan_buf, 0, SLCAN_BUF_SIZE);

	while (rd < nbyte) {
		/* Start reading into the SL ascii buffer */
		uint8_t *pos = (uint8_t *)h->slcan_buf;
		while (1) {
			if (!ReadFile(h->fd, pos, 1, &r, NULL))
				return PORT_ERR_UNKNOWN;

			if (r == 0)
				return PORT_ERR_TIMEDOUT;
			if (pos[0] == '\r')
				break;

			/* Check impossible things */
			if ((pos - h->slcan_buf) > SLCAN_BUF_SIZE)
				return PORT_ERR_UNKNOWN;

			pos++;
		}

		if (slcan_packet_parse(port, ptr, nbyte, &bytes_read) == PORT_ERR_OK) {
			rd += bytes_read;
			ptr += bytes_read;
		} else {
			return PORT_ERR_UNKNOWN;
		}
	}

	return PORT_ERR_OK;
}

static port_err_t slcan_packet_write(struct port_interface *port, uint16_t id, uint8_t dlc, uint8_t *buf)
{
	serial_t *h = (serial_t *)port->private;
	char *ptr = NULL;
	size_t nbyte;
	int i = 0;
	DWORD r;

	if (h == NULL)
		return PORT_ERR_UNKNOWN;

	ptr = h->slcan_buf;
	id &= 0x7FF; /* Strip */
	if (dlc > 8)
		dlc = 8;

	/* Clear it out */
	memset(h->slcan_buf, 0, SLCAN_BUF_SIZE);

	/* Prepare frame */
	snprintf(ptr, SLCAN_BUF_SIZE, "t%03x%d", id, dlc);
	ptr += 5;
	for (i = 0; i < dlc; i++) {
		ptr[0] = "0123456789ABCDEF"[buf[i] >> 4];
		ptr[1] = "0123456789ABCDEF"[buf[i] & 0xF];
		ptr += 2;
	}
	/* Transmission terminator */
	ptr[0] = '\r';
	ptr++;
	/* Null terminate just in case */
	ptr[0] = 0;
	nbyte = ptr - h->slcan_buf;

	if (!WriteFile(h->fd, h->slcan_buf, nbyte, &r, NULL))
		return PORT_ERR_UNKNOWN;
	if (r < nbyte)
		return PORT_ERR_UNKNOWN;

	/* Save transmitted CAN ID for check with RX */
	h->can_id = id;

	return PORT_ERR_OK;
}

static port_err_t slcan_w32_write(struct port_interface *port, void *buf, size_t nbyte)
{
	serial_t *h;
	uint8_t *pos = (uint8_t *)buf;

	h = (serial_t *)port->private;
	if (h == NULL)
		return PORT_ERR_UNKNOWN;

	/* slcan buffer frame is going to be:
	 * [0] = can_id
	 * [1]+ = buffer[nbyte]
	 * dlc = nbyte - 1
	 */

	/* At least ID must be present */
	if (nbyte < 1)
		return PORT_ERR_UNKNOWN;

	return slcan_packet_write(port, pos[0], (uint8_t)(nbyte - 1), pos + 1);
}

static port_err_t slcan_w32_gpio(struct port_interface *port, serial_gpio_t __unused n, int __unused level)
{
	return PORT_ERR_OK;
}

static const char *slcan_w32_get_cfg_str(struct port_interface *port)
{
	serial_t *h;

	h = (serial_t *)port->private;
	return h ? h->setup_str : "INVALID";
}

static port_err_t slcan_w32_flush(struct port_interface *port)
{
	serial_t *h;
	h = (serial_t *)port->private;
	if (h == NULL)
		return PORT_ERR_UNKNOWN;

	slcan_flush(h);

	return PORT_ERR_OK;
}

struct port_interface port_slcan = {
	.name	= "slcan_w32",
	.flags	= PORT_BYTE | PORT_GVR_ETX | PORT_CMD_INIT | PORT_RETRY | PORT_CANBUS,
	.open	= slcan_w32_open,
	.close	= slcan_w32_close,
	.flush  = slcan_w32_flush,
	.read	= slcan_w32_read,
	.write	= slcan_w32_write,
	.gpio	= slcan_w32_gpio,
	.get_cfg_str	= slcan_w32_get_cfg_str,
};
