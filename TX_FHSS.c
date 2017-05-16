/*
Implementation of FHSS algoritm with nRF24L01 and other similar products
Version 1.0 (16/05/2017)
Copyright (C) 2017 Massimo Guercini guercini@gmail.com

This program is free software: you  can redistribute it and/or modify it
under the  terms of the GNU  General Public License as  published by the
Free Software Foundation,  either version 3 of the License,  or (at your
option) any later version.

This  program  is distributed  in  the  hope  that  it will  be  useful,
but  WITHOUT  ANY  WARRANTY;  without   even  the  implied  warranty  of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
Public License for more details.

You should have received a copy  of the GNU General Public License along
with this program. If not, see <http://www.gnu.org/licenses/>.

###########################
!!!!!!! Code for TX !!!!!!!
###########################
*/

// Setup for nRF24L01
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
RF24 radio(6,7);
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };	// Address of PTX and PRX
int interrupt_time = 10; // In millisenconds (Do not overdo it with too short interrupt time)
byte Int_cnt = 0; // Interrupt counter
byte Int_TX_cnt = 5; // Setting up the number of interrupts count that trig the data tramission 
volatile boolean fired = false;

// Channels hopping schema (Each number MUST BE unique in the sequence)
// It's possible to use non unique number in sequence, but in this case we must send the ptr_fhss_schema value to PRX
// thought the data_TX struct to align ptr_fhss_schema of PRX
// However, I prefer the sequence of unique numbers
byte fhss_schema[]={11, 46, 32, 49, 2, 19, 3, 33, 30, 14, 9, 13, 6, 1, 34, 39, 44, 43, 54, 24, 42, 37, 31};	// You can do it as long as you like
byte ptr_fhss_schema = 0; // Pointer to the fhss_schema array

typedef struct{	// Struct of data to send to PRX
	int var1_value;
	int var2_value;
	int var3_value;
	int var4_value;
	byte var5_value;
}
A_t;

typedef struct{	// Struct of data received from PRX
	int var1_value;
	int var2_value;
	int var3_value;
	int var4_value;
	int var5_value;
	int var6_value;
}
B_t;

A_t data_TX;
B_t data_RX;

void setup()
{
	Serial.begin(115200);
	printf_begin();
	radio.begin();
	radio.setPALevel(RF24_PA_LOW); // RF24_PA_MIN (-18dBm), RF24_PA_LOW (-12dBm), RF24_PA_HIGH (-6dBM), RF24_PA_MAX (0dBm)
	radio.setRetries(4,9);
	radio.setAutoAck(1);
	radio.enableAckPayload();
	radio.enableDynamicPayloads();
	radio.setDataRate(RF24_250KBPS);
	radio.setChannel(fhss_schema[ptr_fhss_schema]);
	radio.openWritingPipe(pipes[1]);
	radio.openReadingPipe(1,pipes[0]);
//	Setup some initial data value
	data_TX.var1_value=1;
	data_TX.var2_value=2;
	data_TX.var3_value=3;
	data_TX.var4_value=4;
	data_TX.var5_value=0x05;
	radio.startListening();
	radio.printDetails();
	// Setup interrupt every interrupt_time value
	// CTC mode with clk/8 prescaler
	TCCR1A = 0;
	TCCR1B = 1<<WGM12 | 1<<CS11;
	TCNT1 = 0;         // reset counter
	OCR1A =  (interrupt_time*2000)-1;       // compare A register value 
	TIFR1 |= _BV (OCF1A);    // clear interrupt flag
	TIMSK1 = _BV (OCIE1A);   // interrupt on Compare A Match
}

void loop()
{
	if(fired) {	// When the interrupt occurred, we need to perform the following task  
		fired=false;	// Reset fired flag
		Int_cnt++;	// Increment Interrupts counter
		if(Int_cnt==(Int_TX_cnt-1)) {	// If it's time to perform channel change (10ms before trasmission time)
			ptr_fhss_schema++;	// Increment pointer of fhss schema array to perform next channel change
			if(ptr_fhss_schema >= sizeof(fhss_schema)) ptr_fhss_schema=0;	// To avoid array indexing overflow
			radio.setChannel(fhss_schema[ptr_fhss_schema]);	// Change channel
		}
	}
	if(Int_cnt == Int_TX_cnt) {	// If it's time to transmit.
		radio.stopListening();
		radio.write( &data_TX, sizeof(data_TX) );
		if(radio.isAckPayloadAvailable())	// If we received data in ACK Payload, read and print values.
		{
			radio.read(&data_RX, sizeof(data_RX));
			Serial.print("Data from RX station : ");
			Serial.print(data_RX.var1_value);
			Serial.print(", ");
			Serial.print(data_RX.var2_value);
			Serial.print(", ");
			Serial.print(data_RX.var3_value);
			Serial.print(", ");
			Serial.print(data_RX.var4_value);
			Serial.print(", ");
			Serial.print(data_RX.var5_value);
			Serial.print(", ");
			Serial.println(data_RX.var6_value);
		}
		radio.startListening();
		Int_cnt = 0;
	} else {
	// Put here your code to retrive data to send to PRX
	// I setup some value as example
		data_TX.var1_value++;
		data_TX.var2_value++;
		data_TX.var3_value++;
		data_TX.var4_value++;
		data_TX.var5_value++;
	}
}

ISR (TIMER1_COMPA_vect)
{
	fired = true;
}
