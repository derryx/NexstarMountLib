/*
Arduino Library to talk to the AUX port of Celestron Nexstar compatible mounts.

Copyright 2014 Thomas Peuss <thomas at peuss dot de>

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#ifndef NexstarMount_h
#define NexstarMount_h

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "TinyGPS.h"

#define NexstarMount_VERSION 1

#define MSG_PREAMBLE 0x3b
#define DEVICE_MAINBOARD 0x01
#define DEVICE_HANDCONTROLLER 0x04
#define DEVICE_AZM_CONTROLLER 0x10
#define DEVICE_ALT_CONTROLLER 0x11
#define DEVICE_GPS 0xb0

#define MSGID_GPS_GET_LAT 0x01
#define MSGID_GPS_GET_LONG 0x02
#define MSGID_GPS_GET_DATE 0x03
#define MSGID_GPS_GET_YEAR 0x04
#define MSGID_GPS_GET_SAT_INFO 0x07
#define MSGID_GPS_GET_RCVR_STATUS 0x08
#define MSGID_GPS_GET_TIME 0x33
#define MSGID_GPS_TIME_VALID 0x36
#define MSGID_GPS_LINKED 0x37
#define MSGID_GPS_GET_HW_VER 0x55
#define MSGID_GPS_GET_COMPASS 0xa0
#define MSGID_GPS_GET_VER 0xfe

#define calc_msg_length(payloadLength) (payloadLength+3)

struct nexstar_msg_header {
	uint8_t preamble;
	uint8_t length;
	uint8_t from;
	uint8_t to;
	uint8_t messageid;
};

struct nexstar_msg_struct {
	nexstar_msg_header header;
	uint8_t payload[10];
};

union nexstar_msg_union {
	uint8_t data[sizeof(nexstar_msg_struct)];
	nexstar_msg_struct msg;
};

class NexstarMessageReceiver {
public:
	NexstarMessageReceiver();
	bool process(int data);
	bool isValid();
	nexstar_msg_union* getMessage();
	void reset();

protected:
	bool validate_checksum();

	nexstar_msg_union message;
	uint8_t index;
	uint8_t last_index;
	bool preamble_received;
	bool length_received;
	bool finished;
	bool valid;
	long last_receive_timestamp;
};

class NexstarMessageSender {
public:
	NexstarMessageSender(TinyGPS* _gps, uint8_t _rtsPin, uint8_t _ctsPin);
	nexstar_msg_union* getMessage();
	bool send(SoftwareSerial* serial);
	bool handleMessage(NexstarMessageReceiver* receiver);

protected:
	void handleGetLat();
	void handleGetLong();
	void handleGetDate();
	void handleGetTime();
	void handleGetYear();
	void handleTimeValid();
	void handleGpsLinked();
	void handleSatInfo();
	void handleReceiverStatus();
	void handleGetHardwareVersion();
	void handleGetCompass();
	void handleGetSoftwareVersion();

	// Utility methods
	void degToBytes(float* deg);
	void calc_checksum();
	void pinModeTri(int pin);
	void sendByte(SoftwareSerial* serial,uint8_t b);

	// Fields
	nexstar_msg_union message;
	TinyGPS* gps;
	uint8_t ctsPin;
	uint8_t rtsPin;
};

#endif
