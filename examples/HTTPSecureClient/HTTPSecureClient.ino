/*
 *  HTTPSecureClient.ino - GainSpan WiFi Module Control Program
 *  Copyright 2022 Spresense Users
 *
 *  This work is free software; you can redistribute it and/or modify it under the terms 
 *  of the GNU Lesser General Public License as published by the Free Software Foundation; 
 *  either version 2.1 of the License, or (at your option) any later version.
 *
 *  This work is distributed in the hope that it will be useful, but without any warranty; 
 *  without even the implied warranty of merchantability or fitness for a particular 
 *  purpose. See the GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License along with 
 *  this work; if not, write to the Free Software Foundation, 
 *  Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include <HttpGs2200.h>
#include <TelitWiFi.h>
#include "config.h"

#include <SDHCI.h>
#include <RTC.h>

#define  CONSOLE_BAUDRATE  115200

typedef enum{
	POST=0,
	GET
} DEMO_STATUS_E;

DEMO_STATUS_E httpStat;
char sendData[100];

const uint16_t RECEIVE_PACKET_SIZE = 1500;
uint8_t Receive_Data[RECEIVE_PACKET_SIZE] = {0};

TelitWiFi gs2200;
TWIFI_Params gsparams;
HttpGs2200 theHttpGs2200(&gs2200);
HTTPGS2200_HostParams hostParams;
SDClass theSD;

int count = 0;
bool httpresponse = false;
uint32_t start = 0;

void parse_httpresponse(char *message)
{
	char *p;
	
	if ((p=strstr(message, "200 OK\r\n")) != NULL) {
		ConsolePrintf("Response : %s\r\n", p+8);
	}
}

void post() {
	ConsoleLog("POST Start");
	do {
		httpresponse = theHttpGs2200.config(HTTP_HEADER_TRANSFER_ENCODING, "chunked");
	} while (true != httpresponse);
	
	do {
		/* create https connect */
		httpresponse = theHttpGs2200.connect(true);
	} while (true != httpresponse);

	ConsoleLog("Socket Open");
	snprintf(sendData, sizeof(sendData), "data=%d", count);
	do {
		httpresponse = theHttpGs2200.send(HTTP_METHOD_POST, 10, "/post", sendData, strlen(sendData));
	} while (true != httpresponse);
	
	/* Need to receive the HTTP response */
	while (1) {
		if (gs2200.available()) {
			if (0 < theHttpGs2200.receive(Receive_Data, RECEIVE_PACKET_SIZE)) {
					parse_httpresponse( (char *)(Receive_Data) );
			}
			WiFi_InitESCBuffer();
			break;
		}
	}
	start = millis();
	while (1) {
		if (gs2200.available()) {
			if (true == theHttpGs2200.receive()) {
				// AT+HTTPSEND command is done
				break;
			}
			if (msDelta(start)>20000) {// Timeout
				ConsoleLog("msDelta(start)>20000 Timeout.");
				break;
			}
		}
	}

	do {
		httpresponse = theHttpGs2200.end();
	} while (true != httpresponse);
	ConsoleLog("Socket Closed");
	
	delay(1000);
	httpStat = GET;
	count+=100;
}

void get() {
	ConsoleLog("GET Start");
	do {
		httpresponse = theHttpGs2200.config(HTTP_HEADER_TRANSFER_ENCODING, "identity");
	} while (true != httpresponse);
	
	ConsoleLog("Open Socket");
	do {
		/* create https connect */
		httpresponse = theHttpGs2200.connect(true);
	} while (true != httpresponse);

	httpresponse = theHttpGs2200.send(HTTP_METHOD_GET, 10, HTTP_PATH, "", 0);
	if (true == httpresponse) {
		theHttpGs2200.get_data(Receive_Data, RECEIVE_PACKET_SIZE);
		parse_httpresponse((char *)(Receive_Data));
		WiFi_InitESCBuffer();
	} else {
		ConsoleLog( "?? Unexpected HTTP Response ??" );
	}
	start = millis();
	while (1) {
		if (gs2200.available()) {
			if (false == theHttpGs2200.receive()) {
				theHttpGs2200.get_data(Receive_Data, RECEIVE_PACKET_SIZE);
				ConsolePrintf("%s", (char *)(Receive_Data));
				WiFi_InitESCBuffer();
			} else{
				// AT+HTTPSEND command is done
				ConsolePrintf( "\r\n");
				break;
			}
		}
		if (msDelta(start)>20000) {// Timeout
			ConsoleLog("msDelta(start)>20000 Timeout.");
			break;
		}
	}
 
	do {
		httpresponse = theHttpGs2200.end();
	} while (true != httpresponse);
	ConsoleLog("Socket Closed");
	
	delay(5000);
	httpStat = POST;
}

void setup() {

	/* Initialize SD */
	while (!theSD.begin()) {
		; /* wait until SD card is mounted. */
	}

	RTC.begin();
	RtcTime compiledDateTime(__DATE__, __TIME__);
	RTC.setTime(compiledDateTime);
  
	/* initialize digital pin LED_BUILTIN as an output. */
	pinMode(LED0, OUTPUT);
	digitalWrite(LED0, LOW);   // turn the LED off (LOW is the voltage level)
	Serial.begin(CONSOLE_BAUDRATE); // talk to PC

	/* Initialize SPI access of GS2200 */
	Init_GS2200_SPI_type(iS110B_TypeC);

	/* Initialize AT Command Library Buffer */
	gsparams.mode = ATCMD_MODE_STATION;
	gsparams.psave = ATCMD_PSAVE_DEFAULT;
	if (gs2200.begin(gsparams)) {
		ConsoleLog("GS2200 Initilization Fails");
		while (1);
	}

	/* GS2200 Association to AP */
	if (gs2200.activate_station(AP_SSID, PASSPHRASE)) {
		ConsoleLog("Association Fails");
		while (1);
	}

	hostParams.host = (char *)HTTP_SRVR_IP;
	hostParams.port = (char *)HTTP_PORT;
	theHttpGs2200.begin(&hostParams);

	ConsoleLog("Start HTTP Secure Client");

	/* Set HTTP Headers */
	theHttpGs2200.config(HTTP_HEADER_AUTHORIZATION, "Basic dGVzdDp0ZXN0MTIz");
	theHttpGs2200.config(HTTP_HEADER_TRANSFER_ENCODING, "chunked");
	theHttpGs2200.config(HTTP_HEADER_CONTENT_TYPE, "application/x-www-form-urlencoded");
	theHttpGs2200.config(HTTP_HEADER_HOST, HTTP_SRVR_IP);
	// Set certifications via a file on the SD card before connecting to the server
	File rootCertsFile = theSD.open(ROOTCA_FILE, FILE_READ);

	char time_string[128];
	RtcTime rtc = RTC.getTime();
	snprintf(time_string, sizeof(time_string), "%02d/%02d/%04d,%02d:%02d:%02d", rtc.day(), rtc.month(), rtc.year(), rtc.hour(), rtc.minute(), rtc.second());
  
	theHttpGs2200.set_cert((char*)"TLS_CA", time_string, 0, 1, &rootCertsFile); 
	rootCertsFile.close();
  
	digitalWrite(LED0, HIGH); // turn on LED
}

// the loop function runs over and over again forever
void loop() {
	httpStat = GET;
	while (1) {
		switch (httpStat) {
		case POST:
			post();
			break;
			
		case GET:
			get();
			break;
		default:
			break;
		}
	}
}
