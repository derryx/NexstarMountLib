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

#include "Arduino.h"
#include "NexstarMount.h"

NexstarMessageReceiver::NexstarMessageReceiver() :
		preamble_received(false), length_received(false), finished(false), valid(
				false), index(0), last_index(0), last_receive_timestamp(-1) {
}

union nexstar_msg_union* NexstarMessageReceiver::getMessage() {
	return &message;
}

bool NexstarMessageReceiver::isValid() {
	return valid;
}

bool NexstarMessageReceiver::validate_checksum() {
	if (!finished) {
		return false;
	}
	int result = 0;

	for (int i = 1; i < message.msg.header.length + 2; ++i) {
		result += message.data[i];
	}
	result = -result;

	return (message.data[message.msg.header.length + 2] == (result & 0xff));
}

void NexstarMessageReceiver::reset() {
	index = 0;
	preamble_received = false;
	length_received = false;
	finished = false;
	valid = false;
	last_index = 0;
	last_receive_timestamp = -1;
}

bool NexstarMessageReceiver::process(int data) {
	// Too long delay between uint8_ts?
	unsigned long current_millis=millis();

	if (last_receive_timestamp != -1
			&& current_millis - last_receive_timestamp > 250) {
		reset();
		return false;
	}
	last_receive_timestamp = current_millis;

	if ((!preamble_received) && (data == MSG_PREAMBLE)) {
		index = 0;
		preamble_received = true;
		message.data[index++] = data;
		return false;
	}

	if (preamble_received && (index == 1)) {
		length_received = true;
		message.data[index++] = data;

		// Plausi check
		if (data > 15) {
			reset();
			return false;
		}

		last_index = data + 3;

		return false;
	}

	if (length_received && (index > 1)) {
		message.data[index++] = data;
	}

	// Wir sind fertig
	if (index == last_index) {
		reset();
		finished = true;

		valid = validate_checksum();

		return valid;
	}

	return false;
}

NexstarMessageSender::NexstarMessageSender(TinyGPS* _gps, uint8_t _rtsPin, uint8_t _ctsPin)
: gps(_gps),rtsPin(_rtsPin),ctsPin(_ctsPin)
{
	message.msg.header.preamble = MSG_PREAMBLE;
}

union nexstar_msg_union* NexstarMessageSender::getMessage() {
	return &message;
}

void NexstarMessageSender::calc_checksum() {
	int result = 0;

	for (int i = 1; i < message.msg.header.length + 2; ++i) {
		result += message.data[i];
	}
	result = -result;

	message.data[message.msg.header.length + 2] = (result & 0xff);
}

bool NexstarMessageSender::send(SoftwareSerial* serial) {
	uint8_t bytes_to_send=message.msg.header.length-2;
	calc_checksum();
//	Serial.println("Starting to send response");
//	Serial.println("Checking for RTS to be low");
	long start=millis();
	while(digitalRead(rtsPin)==LOW) {
		// wait
//		Serial.println("Waiting for signals to be released");
		if (millis()-start>250) {
			goto ende;
		}
	}
	// switch to OUTPUT FOR rtsPin
	digitalWrite(rtsPin,HIGH);
	pinMode(rtsPin,OUTPUT);
	
	// Request send allowance
	digitalWrite(rtsPin,LOW);
	
	// Wait for send clearance
	start=millis();
	while(digitalRead(ctsPin)==HIGH) {
//		Serial.println("Waiting for CTS");
		if (millis()-start>250) {
			goto ende;
		}
	}

	sendByte(serial,message.msg.header.preamble);
	sendByte(serial,message.msg.header.length);
	sendByte(serial,message.msg.header.from);
	sendByte(serial,message.msg.header.to);
	sendByte(serial,message.msg.header.messageid);
	
	for (uint8_t c=0;c<bytes_to_send;++c) {
		sendByte(serial,message.msg.payload[c]);
	}

	digitalWrite(rtsPin,HIGH);
	ende:
	pinModeTri(rtsPin);
	return true;
}

inline void NexstarMessageSender::sendByte(SoftwareSerial* serial,uint8_t b)
{
	serial->write(b);
}

inline void NexstarMessageSender::pinModeTri(int pin) {
	  digitalWrite(pin,LOW);
	  pinMode(pin,OUTPUT);
	  pinMode(pin,INPUT);
}

bool NexstarMessageSender::handleMessage(NexstarMessageReceiver* receiver)
{
	  nexstar_msg_union* msgin=receiver->getMessage();
	  
	  // for us?
	  if (msgin->msg.header.to!=DEVICE_GPS) {
	      // ignore
	      return false;
	  }

	  // OK now we have work
	  union nexstar_msg_union* msgout=getMessage();

	  msgout->msg.header.from=DEVICE_GPS;
	  msgout->msg.header.to=msgin->msg.header.from;
	  msgout->msg.header.messageid=msgin->msg.header.messageid;
	  
	  switch (msgin->msg.header.messageid) {
	    case MSGID_GPS_GET_LAT:
	      handleGetLat();
	      break;
	    case MSGID_GPS_GET_LONG:
	      handleGetLong();
	      break;
	    case MSGID_GPS_GET_DATE:
	      handleGetDate();
	      break;
	    case MSGID_GPS_GET_YEAR:
	      handleGetYear();
	      break;
	    case MSGID_GPS_GET_SAT_INFO:
	      handleSatInfo();
	      break;
	    case MSGID_GPS_GET_RCVR_STATUS:
	      handleReceiverStatus();
	      break;
	    case MSGID_GPS_GET_TIME:
	      handleGetTime();
	      break;
	    case MSGID_GPS_TIME_VALID:
	      handleTimeValid();
	      break;
	    case MSGID_GPS_LINKED:
	      handleGpsLinked();
	      break;
	    case MSGID_GPS_GET_HW_VER:
	      handleGetHardwareVersion();
	      break;
	    case MSGID_GPS_GET_COMPASS:
	      handleGetCompass();
	      break;
	    case MSGID_GPS_GET_VER:
	      handleGetSoftwareVersion();
	      break;
	    default:
	      return false;
	  }
	  
	  // We would respond...
//	      Serial.println("Message prepared");
//	      Serial.print("From: ");
//	      Serial.println(msgout->msg.from,HEX);
//	      Serial.print("To: ");
//	      Serial.println(msgout->msg.to,HEX);
//	      Serial.print("Length: ");
//	      Serial.println(msgout->msg.length,HEX);
//	      Serial.print("MessageId: ");
//	      Serial.println(msgout->msg.messageid,HEX);
//	      Serial.println("Data:");
//	      for (int i=0;i<msgout->msg.length-2;++i) {
//	        Serial.print(msgout->msg.payload[i],HEX);
//	        Serial.print(",");
//	      }
//	      Serial.println();  
	      return true;
}

inline void NexstarMessageSender::handleGetLat()
{
	float latitude,longitude;
	unsigned long fix_age;
	
	message.msg.header.length=calc_msg_length(3);
	gps->f_get_position(&latitude,&longitude,&fix_age);
	degToBytes(&latitude);
}

inline void NexstarMessageSender::handleGetLong()
{
	float latitude,longitude;
	unsigned long fix_age;
	
	message.msg.header.length=calc_msg_length(3);
	gps->f_get_position(&latitude,&longitude,&fix_age);
	degToBytes(&longitude);
}

inline void NexstarMessageSender::handleGetDate()
{
	uint8_t month,day;
	
	message.msg.header.length=calc_msg_length(2);
	gps->crack_datetime(0,&month,&day,0,0,0,0);
	message.msg.payload[0]=month;
	message.msg.payload[1]=day;
}

inline void NexstarMessageSender::handleGetTime()
{
	uint8_t hour,minute,second;
	
	message.msg.header.length=calc_msg_length(3);
	gps->crack_datetime(0,0,0,&hour,&minute,&second,0);
	
	message.msg.payload[0]=hour;
	message.msg.payload[1]=minute;
	message.msg.payload[2]=second;
}

inline void NexstarMessageSender::handleGetYear()
{
	int year;
	
	message.msg.header.length=calc_msg_length(2);
	gps->crack_datetime(&year,0,0,0,0,0,0);
	
	message.msg.payload[0]=(year>>8)&0xFF;
	message.msg.payload[1]=year&0xFF;
}

inline void NexstarMessageSender::handleTimeValid()
{
	unsigned long fix_age;
	
	message.msg.header.length=calc_msg_length(1);
	gps->get_datetime(NULL,NULL,&fix_age);
	
	if (fix_age==gps->GPS_INVALID_AGE || fix_age>5000) {
		message.msg.payload[0]=0x00;
	} else {
		message.msg.payload[0]=0x01;
	}
}

inline void NexstarMessageSender::handleGpsLinked()
{
	unsigned long fix_age;
	
	message.msg.header.length=calc_msg_length(1);
	gps->get_position(0,0,&fix_age);
	
	if (fix_age==gps->GPS_INVALID_AGE || fix_age>5000) {
		message.msg.payload[0]=0x00;
	} else {
		message.msg.payload[0]=0x01;
	}
}

inline void NexstarMessageSender::handleSatInfo()
{
   message.msg.header.length=calc_msg_length(2);
   message.msg.payload[0]=gps->satellites()&0xFF; //Visible Satellites
   message.msg.payload[1]=gps->satellites()&0xFF; // Tracked Satellites
}

inline void NexstarMessageSender::handleReceiverStatus()
{
   message.msg.header.length=calc_msg_length(2);
   message.msg.payload[0]=B11100000; // uint8_t 1
   message.msg.payload[1]=B00010000; // uint8_t 0
}

inline void NexstarMessageSender::handleGetHardwareVersion()
{
	// Fake implementation always returning 0xAB
    message.msg.header.length=calc_msg_length(1);
    message.msg.payload[0]=0xAB;  // FIX MODEL
}

inline void NexstarMessageSender::handleGetCompass()
{
	// Fake implementation always returning NORTH
	message.msg.header.length=calc_msg_length(1);
	message.msg.payload[0]=0x0B;  // NORTH=0x0b
}

inline void NexstarMessageSender::handleGetSoftwareVersion()
{
	message.msg.header.length=calc_msg_length(2);
	message.msg.payload[0]=0x01; //Major Version
	message.msg.payload[1]=0x00; //Minor Version
}

inline void NexstarMessageSender::degToBytes(float* deg)
{
	*deg=(*deg*0xFFFFFF)/360;
	long ldeg=long(*deg);
	
	message.msg.payload[0]=(ldeg>>16)&0xFF;
	message.msg.payload[1]=(ldeg>>8)&0xFF;
	message.msg.payload[2]=ldeg&0xFF;
}
