/*
 *  SPRESENSE_WiFi.ino - GainSpan WiFi Module Control Program
 *  Copyright 2019 Norikazu Goto
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

#include <GS2200Hal.h>
#include <GS2200AtCmd.h>
#include <TelitWiFi.h>
#include "config.h"

#define  CONSOLE_BAUDRATE  115200
/*-------------------------------------------------------------------------*
 * Globals:
 *-------------------------------------------------------------------------*/
extern uint8_t ESCBuffer[];
extern uint32_t ESCBufferCnt;
TelitWiFi gs2200;
TWIFI_Params gsparams;
char UDP_Data[] = "GS2200 UDP Client Data Transfer Test.";
const uint16_t UDP_PACKET_SIZE = 37; 

/*-------------------------------------------------------------------------*
 * Function ProtoTypes:
 *-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*
 * led_onoff
 *---------------------------------------------------------------------------*/
static void led_onoff(int num, bool stat)
{
	switch (num) {
	case 0:
		digitalWrite(LED0, stat);
		break;

	case 1:
		digitalWrite(LED1, stat);
		break;

	case 2:
		digitalWrite(LED2, stat);
		break;

	case 3:
		digitalWrite(LED3, stat);
		break;
	}

}

/*---------------------------------------------------------------------------*
 * led_effect
 *---------------------------------------------------------------------------*
 * Description: See this effect....
 *---------------------------------------------------------------------------*/
static void led_effect(void)
{
	static int cur=0;
	int i;
	static bool direction=true; // which way to go
	

	for (i=-1; i<5; i++) {
		if (i==cur) {
			led_onoff(i, true);
			if (direction)
				led_onoff(i-1, false);
			else
				led_onoff(i+1, false);
		}
	}

	if (direction) { // 0 -> 1 -> 2 -> 3
		if (++cur > 4)
			direction = false;
	}
	else {
		if (--cur < -1)
			direction = true;
	}
		
}

// the setup function runs once when you press reset or power the board
void setup() {
	/* initialize digital pin of LEDs as an output. */
	pinMode(LED0, OUTPUT);
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
	pinMode(LED3, OUTPUT);

	digitalWrite(LED0, LOW);   // turn the LED off (LOW is the voltage level)
	Serial.begin(CONSOLE_BAUDRATE); // talk to PC

	/* Initialize AT Command Library Buffer */
	AtCmd_Init();
	/* Initialize SPI access of GS2200 */
	Init_GS2200_SPI_type(iS110B_TypeC);
	/* Initialize AT Command Library Buffer */
	gsparams.mode = ATCMD_MODE_STATION;
	gsparams.psave = ATCMD_PSAVE_DEFAULT;
	if (gs2200.begin(gsparams)) {
		ConsoleLog("GS2200 Initilization Fails");
		while(1);
	}

	/* GS2200 Association to AP */
	if (gs2200.activate_station(AP_SSID, PASSPHRASE)) {
		ConsoleLog("Association Fails");
		while(1);
	}
	digitalWrite(LED0, HIGH); // turn on LED

}

// the loop function runs over and over again forever
void loop() {
	ATCMD_RESP_E resp;
	char server_cid = 0;
	bool served = false;
	uint32_t timer = 0;
	while (1) {
		if (!served) {
			ATCMD_NetworkStatus networkStatus;
			resp = ATCMD_RESP_UNMATCH;

			ConsoleLog("Start UDP Client");
			resp = AtCmd_NCUDP((char *)UDPSRVR_IP, (char *)UDPSRVR_PORT, (char *)LocalPort, &server_cid);   // Create UDP Client; AT+NCUDP=<Dest-Address>,<Port>[<,Src-Port>]
			ConsolePrintf("server_cid: %d \r\n", server_cid);

			if (resp != ATCMD_RESP_OK) {
				ConsoleLog("No Connect!");
				delay(2000);
				continue;
			}
			if (server_cid == ATCMD_INVALID_CID) {
				ConsoleLog("No CID!");
				delay(2000);
				continue;
			}

			do {
				resp = AtCmd_NSTAT(&networkStatus);         // AT+NSTAT=?
			} while (ATCMD_RESP_OK != resp);

			ConsoleLog("Connected");
			ConsolePrintf("IP: %d.%d.%d.%d\r\n\r\n",
						networkStatus.addr.ipv4[0], networkStatus.addr.ipv4[1], networkStatus.addr.ipv4[2], networkStatus.addr.ipv4[3]);

			served = true;
		}
		else {
			ConsoleLog("Start to send UDP Data");

			// Prepare for the next chunck of incoming data
			WiFi_InitESCBuffer();

			ConsolePrintf("\r\n");

			while (1) {
				AtCmd_SendBulkData(server_cid, UDP_Data, UDP_PACKET_SIZE);

				resp = AtCmd_RecvResponse();  // Description: Wait for a response after sending a command. Keep parsing the data until a response is found.

				if (ATCMD_RESP_BULK_DATA_RX == resp) {          
					if (Check_CID(server_cid)) {
						ConsolePrintf("%d byte Recieved successfully. \r\n", ESCBufferCnt);

						for (int i = 1; i < ESCBufferCnt; i++) {
							ConsolePrintf("%c", ESCBuffer[i]);
						}
						ConsolePrintf("\r\n");
					}
					WiFi_InitESCBuffer();

					delay(100);
				}

				if (msDelta(timer) > 100) {
					timer = millis();
					led_effect();
				}
			}
		}
	}
}
