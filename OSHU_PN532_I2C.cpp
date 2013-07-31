/**************************************************************************/
/*! 
    @file     PN532_I2C.cpp
    @author   teuteuguy
	@license  
	
	@section  HISTORY

    v0.1 - Creation

*/
/**************************************************************************/
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Wire.h>
#ifdef __AVR__
 #define WIRE Wire
#else // Arduino Due
 #define WIRE Wire1
#endif


#include "OSHU_PN532_I2C.h"

//#define OSHU_PN532_I2C_DEBUG
//#define OSHU_PN532_LOWLEVEL_DEBUG
//#define OSHU_PN532_EZLINK_DEBUG

byte oshu_pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
byte oshu_pn532response_firmwarevers[] = {0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03};

#define OSHU_PN532_PACKBUFFSIZE 64
byte oshu_pn532_packetbuffer[OSHU_PN532_PACKBUFFSIZE];

// volatile uint8_t _ezlink[8];// = { 0, 0, 0, 0, 0, 0, 0, 0 };

// void pn532it(); // Interrupt routine.

/**************************************************************************/
/*! 
    @brief  Sends a single byte via I2C

    @param  x    The byte to send
*/
/**************************************************************************/
static inline void wiresend(uint8_t x) 
{
  #if ARDUINO >= 100
    WIRE.write((uint8_t)x);
  #else
    WIRE.send(x);
  #endif
}

/**************************************************************************/
/*! 
    @brief  Reads a single byte via I2C
*/
/**************************************************************************/
static inline uint8_t wirerecv(void) 
{
  #if ARDUINO >= 100
    return WIRE.read();
  #else
    return WIRE.receive();
  #endif
}







/**************************************************************************/
/*! 
    @brief  Creates a OSHU_PN532_I2C class

    @param  irq       Location of the IRQ pin
    @param  reset     Location of the RSTPD_N pin
*/
/**************************************************************************/
OSHU_PN532_I2C::OSHU_PN532_I2C(uint8_t irq, uint8_t reset, bool irqmode) {
	_irq = irq;
	_irqmode = irqmode;
	_reset = reset;
	pinMode(_irq, INPUT);
	pinMode(_reset, OUTPUT);
}


/**************************************************************************/
/*! 
    @brief  Initializes the OSHU_PN532 hardware (I2C)
*/
/**************************************************************************/
bool OSHU_PN532_I2C::init( ) {

	WIRE.begin();

	// Reset the PN532  
	digitalWrite(_reset, HIGH);
	digitalWrite(_reset, LOW);
	delay(400);
	digitalWrite(_reset, HIGH);

	uint32_t versiondata;

	oshu_pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;

	if (! sendCommandCheckAck(oshu_pn532_packetbuffer, 1) ) return 0;

	// read data packet
	wirereaddata(oshu_pn532_packetbuffer, 12);

	// check some basic stuff
	if (0 != strncmp((char *)oshu_pn532_packetbuffer, (char *)oshu_pn532response_firmwarevers, 6)) {
		#ifdef OSHU_PN532_LOWLEVEL_DEBUG
			Serial.println("Firmware doesn't match!");
		#endif
		return 0;
	}

	versiondata = oshu_pn532_packetbuffer[7];
	versiondata <<= 8;
	versiondata |= oshu_pn532_packetbuffer[8];
	versiondata <<= 8;
	versiondata |= oshu_pn532_packetbuffer[9];
	versiondata <<= 8;
	versiondata |= oshu_pn532_packetbuffer[10];

	if (! versiondata ) {
		#ifdef OSHU_PN532_I2C_DEBUG
			Serial.println("OSHU_PN532_I2C::init: OSHU_PN532 Board not connected.");
		#endif
		//while (1); // halt
		return false;
	}
	#ifdef OSHU_PN532_I2C_DEBUG
		Serial.print("OSHU_PN532_I2C::init: PN5"); Serial.print((versiondata>>24) & 0xFF, HEX);
		Serial.print(" chip (Firmware ver. ");
		Serial.print((versiondata>>16) & 0xFF, DEC);
		Serial.print('.'); Serial.print((versiondata>>8) & 0xFF, DEC);
		Serial.println(") found.");
	#endif


	oshu_pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
	oshu_pn532_packetbuffer[1] = 0x01; // normal mode;
	oshu_pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
	oshu_pn532_packetbuffer[3] = 0x01; // use IRQ pin!

	if (! sendCommandCheckAck(oshu_pn532_packetbuffer, 4))
	return false;

	// read data packet
	wirereaddata(oshu_pn532_packetbuffer, 8);

	return  (oshu_pn532_packetbuffer[6] == 0x15 );//&& setupIRQMode());

}

// /**************************************************************************/
// /*! 
//     @brief  function to setup IRQ system for the library
// */
// /**************************************************************************/
// bool OSHU_PN532_I2C::setupIRQMode( ) {
// 	uint8_t inter = 0;

// 	if (_irqmode) {

// 		switch (_irq) {
// 			case 2:
// 				inter = 0;
// 				break;
// 			case 3:
// 				inter = 1;
// 				break;
// 			case 21:
// 				inter = 2;
// 				break;
// 			case 20:
// 				inter = 3;
// 				break;
// 			case 19:
// 				inter = 4;
// 				break;
// 			case 18:
// 				inter = 5;
// 				break;
// 			default:
// 				return false;
// 				break;
// 		}

// 	}

// 	if ( ! setupPN532ToDetectEZLinkCards() ) {

// 		return false;

// 	}

// 	attachInterrupt(inter, pn532it, RISING);

// 	return true;
// }



// void pn532it() {

// 	Serial.println("IT");
// /*
// 	float balance;

// 	if ( readEZLink( (uint8_t *)_ezlink, &balance ) ) {

// 		//ezLinkAvailable = true;

// 	} else {

// 		//ezLinkAvailable = false;

// 	}

// 	setupPN532ToDetectEZLinkCards();
// */
// }

/**************************************************************************/
/*! 
    @brief  Utility function to printout for debug
*/
/**************************************************************************/
#ifdef OSHU_PN532_EZLINK_DEBUG
char * print8bitHex(int num) {
	char tmp[16];
	char format[128];
    sprintf(format, "%%.%dX", 2);
	sprintf(tmp, format, num);
	return tmp;
}
#endif
/*
void OSHU_PN532_I2C::getEZLink(uint8_t * ezlink) {

	memcpy(ezlink, _ezlink, 8);
	ezLinkAvailable = false;

}
*/

bool OSHU_PN532_I2C::setupPN532ToDetectEZLinkCards() {
	oshu_pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
	oshu_pn532_packetbuffer[1] = 0x01;
	oshu_pn532_packetbuffer[2] = 0x03;
	oshu_pn532_packetbuffer[3] = 0x00;

	#ifdef OSHU_PN532_EZLINK_DEBUG 
		Serial.println("OSHU_PN532_I2C::setupPN532ToDetectEZLinkCards: Searching for EZLink Cards around");
	#endif
	
	if ( ! sendCommandCheckAck(oshu_pn532_packetbuffer, 4, 1000) ) {
		
		#ifdef OSHU_PN532_EZLINK_DEBUG
			Serial.println("OSHU_PN532_I2C::setupPN532ToDetectEZLinkCards: ERROR - NP532 is not responding - ACK timedout");
		#endif

		return false;

	} else {
		// ACK received from PN532.
		// Must wait now for IRQ to trigger.
		return true;
	}
}

bool OSHU_PN532_I2C::checkResponse( uint8_t expectedresponse, uint8_t extraparam ) {
	if (
		oshu_pn532_packetbuffer[0] == 0x00 &&
		oshu_pn532_packetbuffer[1] == 0x00 &&
		oshu_pn532_packetbuffer[2] == 0xFF &&
		oshu_pn532_packetbuffer[4] == (uint8_t)(~oshu_pn532_packetbuffer[3] + 1) &&
		oshu_pn532_packetbuffer[5] == PN532_PN532TOHOST &&
		oshu_pn532_packetbuffer[6] == expectedresponse &&
		oshu_pn532_packetbuffer[7] == extraparam
		) {
		return true;
	}

	return false;

}

bool OSHU_PN532_I2C::readEZLink(uint8_t * ezlink, float * balance) {
	oshu_pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
	oshu_pn532_packetbuffer[1] = 0x01;
	oshu_pn532_packetbuffer[2] = 0x90;//144;//90;
	oshu_pn532_packetbuffer[3] = 0x32;//50;//32;
	oshu_pn532_packetbuffer[4] = 0x03;
	oshu_pn532_packetbuffer[5] = 0x00;
	oshu_pn532_packetbuffer[6] = 0x00;
	oshu_pn532_packetbuffer[7] = 0x00;
				
	#ifdef OSHU_PN532_EZLINK_DEBUG 
		Serial.println("OSHU_PN532_I2C::checkForEZLink: Request read EZLink Card data");
	#endif
			
	if ( ! sendCommandCheckAck(oshu_pn532_packetbuffer, 8, 1000) ) {

		#ifdef OSHU_PN532_EZLINK_DEBUG
			Serial.println("OSHU_PN532_I2C::checkForEZLink: ERROR - NP532 is not responding - ACK timedout");
		#endif

		return false;

	} else {

		if ( ! waitUntilReady(1000) ) {

			#ifdef OSHU_PN532_EZLINK_DEBUG
				Serial.println("OSHU_PN532_I2C::checkForEZLink: ERROR - NP532 is not responding - Fail on IRQ");
			#endif

			return false;

		} else {

			wirereaddata(oshu_pn532_packetbuffer, OSHU_PN532_PACKBUFFSIZE);
			
			if ( ! checkResponse( PN532_RESPONSE_INDATAEXCHANGE, 0x00 ) ) {

				#ifdef OSHU_PN532_EZLINK_DEBUG
					Serial.println("OSHU_PN532_I2C::checkForEZLink: ERROR - Unexpected data read from EZLink.");
				#endif

				return false;

			} else {
								
				#ifdef OSHU_PN532_EZLINK_DEBUG
					Serial.println("OSHU_PN532_I2C::checkForEZLink: Correct response received.");
				#endif

				memcpy(ezlink, oshu_pn532_packetbuffer + 16, 8);
				*balance = (float)(oshu_pn532_packetbuffer[11] * 256 + oshu_pn532_packetbuffer[12]) / 100;
							
				#ifdef OSHU_PN532_EZLINK_DEBUG
					Serial.print("OSHU_PN532_I2C::checkForEZLink: EZLink CAN:");
					for (int i = 0; i < 8; i ++) {
						Serial.print(" "); Serial.print(print8bitHex(ezlink[i]));
					}
					Serial.print(" with balance of ");
					Serial.println(*balance);
				#endif
							
				oshu_pn532_packetbuffer[0] = PN532_COMMAND_INRELEASE;
				oshu_pn532_packetbuffer[1] = 0x01;
			
				#ifdef OSHU_PN532_EZLINK_DEBUG 
					Serial.println("OSHU_PN532_I2C::checkForEZLink: Request release of EZLink Card");
				#endif
			
				if ( ! sendCommandCheckAck(oshu_pn532_packetbuffer, 2, 1000) ) {
				
					#ifdef OSHU_PN532_EZLINK_DEBUG
						Serial.println("OSHU_PN532_I2C::checkForEZLink: ERROR - NP532 is not responding - ACK timedout");
					#endif

					return false;
							
				} else {
							
					if ( ! waitUntilReady(1000) ) {

						#ifdef OSHU_PN532_EZLINK_DEBUG
							Serial.println("OSHU_PN532_I2C::checkForEZLink: ERROR - NP532 is not responding - Fail on IRQ");
						#endif

						return false;

					} else {

						wirereaddata(oshu_pn532_packetbuffer, sizeof(oshu_pn532_packetbuffer));
			
						if ( ! checkResponse( PN532_RESPONSE_INRELEASE, 0x00 ) ) {

							#ifdef OSHU_PN532_EZLINK_DEBUG
								Serial.println("OSHU_PN532_I2C::checkForEZLink: ERROR - NP532 wrong response, but we don't care");
							#endif

						} else {

							#ifdef OSHU_PN532_EZLINK_DEBUG
								Serial.println("OSHU_PN532_I2C::checkForEZLink: Correct release response received.");
								Serial.println(*balance);
							#endif

						}

						return true;

					}
							
				}

			}

		}

	}
}



/**************************************************************************/
/*! 
    @brief  

    @param  
    @param  
*/
/**************************************************************************/
bool OSHU_PN532_I2C::checkForEZLink_NonBlocking(uint8_t * ezlink, float * balance) {

	static uint8_t checkForEZLink_state = 0;

	switch ( checkForEZLink_state ) {
		case 0: {
			// First call => Init
			// Setup the reader to IRQ on detection of a card.
			if ( setupPN532ToDetectEZLinkCards() ) {
				checkForEZLink_state = 1;
			} else {
				checkForEZLink_state = 0;
			}
			} break;
		case 1: {
			// Temporary state to wait for IRQ to be raised.
			// Wait for NP532 to detect something that could be an EZLink
			// This creates the non-blocking effect.
			if ( wirereadstatus() == PN532_I2C_READY ) {
				checkForEZLink_state = 2;
			}
			} break;
		case 2: {
			// IRQ was triggered.
			// Check for EZLink, if fail, revert to state 0.
			wirereaddata(oshu_pn532_packetbuffer, OSHU_PN532_PACKBUFFSIZE);

			if ( ! checkResponse( PN532_RESPONSE_INLISTPASSIVETARGET, 0x01 ) ) {

				#ifdef OSHU_PN532_EZLINK_DEBUG
					Serial.println("OSHU_PN532_I2C::checkForEZLink: NP532 found something else than EZLink or more than 1");
				#endif

				checkForEZLink_state = 0;

			}
			else {
				#ifdef OSHU_PN532_EZLINK_DEBUG
					Serial.println("OSHU_PN532_I2C::checkForEZLink: NP532 found an EZLink");
				#endif
				
				checkForEZLink_state = 0;

				if ( readEZLink(ezlink, balance) ) {
					return true;
				}
/*
				oshu_pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
				oshu_pn532_packetbuffer[1] = 0x01;
				oshu_pn532_packetbuffer[2] = 0x90;//144;//90;
				oshu_pn532_packetbuffer[3] = 0x32;//50;//32;
				oshu_pn532_packetbuffer[4] = 0x03;
				oshu_pn532_packetbuffer[5] = 0x00;
				oshu_pn532_packetbuffer[6] = 0x00;
				oshu_pn532_packetbuffer[7] = 0x00;
				
				#ifdef OSHU_PN532_EZLINK_DEBUG 
					Serial.println("OSHU_PN532_I2C::checkForEZLink: Request read EZLink Card data");
				#endif
				
				if ( ! sendCommandCheckAck(oshu_pn532_packetbuffer, 8, 1000) ) {

					#ifdef OSHU_PN532_EZLINK_DEBUG
						Serial.println("OSHU_PN532_I2C::checkForEZLink: ERROR - NP532 is not responding - ACK timedout");
					#endif

					checkForEZLink_state = 0;

				} else {

					if ( ! waitUntilReady(1000) ) {

						#ifdef OSHU_PN532_EZLINK_DEBUG
							Serial.println("OSHU_PN532_I2C::checkForEZLink: ERROR - NP532 is not responding - Fail on IRQ");
						#endif

						checkForEZLink_state = 0;

					} else {

						wirereaddata(oshu_pn532_packetbuffer, OSHU_PN532_PACKBUFFSIZE);
			
						if ( !(
							oshu_pn532_packetbuffer[0] == 0x00 &&
							oshu_pn532_packetbuffer[1] == 0x00 &&
							oshu_pn532_packetbuffer[2] == 0xFF &&
							oshu_pn532_packetbuffer[4] == (uint8_t)(~oshu_pn532_packetbuffer[3] + 1) &&
							oshu_pn532_packetbuffer[5] == PN532_PN532TOHOST &&
							oshu_pn532_packetbuffer[6] == PN532_RESPONSE_INDATAEXCHANGE &&
							oshu_pn532_packetbuffer[7] == 0x00
							) ) {

							#ifdef OSHU_PN532_EZLINK_DEBUG
								Serial.println("OSHU_PN532_I2C::checkForEZLink: ERROR - Unexpected data read from EZLink.");
							#endif

							checkForEZLink_state = 0;

						} else {
								
							#ifdef OSHU_PN532_EZLINK_DEBUG
								Serial.println("OSHU_PN532_I2C::checkForEZLink: Correct response received.");
							#endif

							memcpy(ezlink, oshu_pn532_packetbuffer + 16, 8);
							*balance = (float)(oshu_pn532_packetbuffer[11] * 256 + oshu_pn532_packetbuffer[12]) / 100;
							
							#ifdef OSHU_PN532_EZLINK_DEBUG
								Serial.print("OSHU_PN532_I2C::checkForEZLink: EZLink CAN:");
								for (int i = 0; i < 8; i ++) {
									Serial.print(" "); Serial.print(print8bitHex(ezlink[i]));
								}
								Serial.print(" with balance of ");
								Serial.println(*balance);
							#endif
							
							oshu_pn532_packetbuffer[0] = PN532_COMMAND_INRELEASE;
							oshu_pn532_packetbuffer[1] = 0x01;
			
							#ifdef OSHU_PN532_EZLINK_DEBUG 
								Serial.println("OSHU_PN532_I2C::checkForEZLink: Request release of EZLink Card");
							#endif
			
							if ( ! sendCommandCheckAck(oshu_pn532_packetbuffer, 2, 1000) ) {
							
								#ifdef OSHU_PN532_EZLINK_DEBUG
									Serial.println("OSHU_PN532_I2C::checkForEZLink: ERROR - NP532 is not responding - ACK timedout");
								#endif

								checkForEZLink_state = 0;
							
							} else {
							
								if ( ! waitUntilReady(1000) ) {

									#ifdef OSHU_PN532_EZLINK_DEBUG
										Serial.println("OSHU_PN532_I2C::checkForEZLink: ERROR - NP532 is not responding - Fail on IRQ");
									#endif

									checkForEZLink_state = 0;

								} else {

									wirereaddata(oshu_pn532_packetbuffer, sizeof(oshu_pn532_packetbuffer));
			
									if ( !(
										oshu_pn532_packetbuffer[0] == 0x00 &&
										oshu_pn532_packetbuffer[1] == 0x00 &&
										oshu_pn532_packetbuffer[2] == 0xFF &&
										oshu_pn532_packetbuffer[4] == (uint8_t)(~oshu_pn532_packetbuffer[3] + 1) &&
										oshu_pn532_packetbuffer[5] == PN532_PN532TOHOST &&
										oshu_pn532_packetbuffer[6] == PN532_RESPONSE_INRELEASE &&
										oshu_pn532_packetbuffer[7] == 0x00
										) ) {

										#ifdef OSHU_PN532_EZLINK_DEBUG
											Serial.println("OSHU_PN532_I2C::checkForEZLink: ERROR - NP532 wrong response, but we don't care");
										#endif

									} else {

										#ifdef OSHU_PN532_EZLINK_DEBUG
											Serial.println("OSHU_PN532_I2C::checkForEZLink: Correct release response received.");
											Serial.println(*balance);
										#endif

									}

									checkForEZLink_state = 0;

									return true;

								}
							
							}

						}

					}

				}

				*/

			}

			} break;
		default: {

			//#ifdef OSHU_PN532_EZLINK_DEBUG
				Serial.println("OSHU_PN532_I2C::checkForEZLink: MAJOR ERROR - Default Case, we should never be here.");
			//#endif
			checkForEZLink_state = 0;

			} break;
	}

	return false;

}



/**************************************************************************/
/*! 
    @brief  Sends a command and waits a specified period for the ACK

    @param  cmd       Pointer to the command buffer
    @param  cmdlen    The size of the command in bytes 
    @param  timeout   timeout before giving up
    
    @returns  1 if everything is OK, 0 if timeout occured before an
              ACK was recieved
*/
/**************************************************************************/
// default timeout of one second
bool OSHU_PN532_I2C::sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen, uint16_t timeout) {
	uint16_t timer = 0;

	// write the command
	wiresendcommand(cmd, cmdlen);

	if ( ! waitUntilReady(timeout) ) {
		return false;
	}

	#ifdef OSHU_PN532_LOWLEVEL_DEBUG
		Serial.println("IRQ received");
	#endif

	// read acknowledgement
	if ( ! readackframe() ) {
		#ifdef OSHU_PN532_LOWLEVEL_DEBUG
			Serial.println("No ACK frame received!");
		#endif
		return false;
	}

	return true; // ack'd command
}

/**************************************************************************/
/*! 
    @brief  Tries to read the PN532 ACK frame (not to be confused with 
          the I2C ACK signal)
*/
/**************************************************************************/
bool OSHU_PN532_I2C::readackframe(void) {
  uint8_t ackbuff[6];
  
  wirereaddata(ackbuff, 6);
    
  return (0 == strncmp((char *)ackbuff, (char *)oshu_pn532ack, 6));
}

/**************************************************************************/
/*! 
    @brief  Checks the IRQ pin to know if the PN532 is ready
  
  @returns 0 if the PN532 is busy, 1 if it is free
*/
/**************************************************************************/
uint8_t OSHU_PN532_I2C::wirereadstatus(void) {
  uint8_t x = digitalRead(_irq);
  
  if (x == 1)
    return PN532_I2C_BUSY;
  else
    return PN532_I2C_READY;
}

/**************************************************************************/
/*! 
    @brief  Reads n bytes of data from the PN532 via I2C

    @param  buff      Pointer to the buffer where data will be written
    @param  n         Number of bytes to be read
*/
/**************************************************************************/
void OSHU_PN532_I2C::wirereaddata(uint8_t* buff, uint8_t n) {
	uint16_t timer = 0;

	delay(2); 

	#ifdef OSHU_PN532_LOWLEVEL_DEBUG
		Serial.print("Reading: ");
	#endif
	// Start read (n+1 to take into account leading 0x01 with I2C)
	WIRE.requestFrom((uint8_t)PN532_I2C_ADDRESS, (uint8_t)(n+2));
	// Discard the leading 0x01
	wirerecv();
	for (uint8_t i=0; i<n; i++) {
		delay(1);
		buff[i] = wirerecv();
		#ifdef OSHU_PN532_LOWLEVEL_DEBUG
			Serial.print(" 0x");
			Serial.print(buff[i], HEX);
		#endif
	}
	// Discard trailing 0x00 0x00
	// wirerecv();

	#ifdef OSHU_PN532_LOWLEVEL_DEBUG
		Serial.println();
	#endif
}

/**************************************************************************/
/*! 
    @brief  Writes a command to the PN532, automatically inserting the
            preamble and required frame details (checksum, len, etc.)

    @param  cmd       Pointer to the command buffer
    @param  cmdlen    Command length in bytes 
*/
/**************************************************************************/
void OSHU_PN532_I2C::wiresendcommand(uint8_t* cmd, uint8_t cmdlen) {
	uint8_t checksum;

	cmdlen++;

	#ifdef OSHU_PN532_LOWLEVEL_DEBUG
		Serial.print("\nSending: ");
	#endif

	delay(2);     // or whatever the delay is for waking up the board

	// I2C START
	WIRE.beginTransmission(PN532_I2C_ADDRESS);
	checksum = PN532_PREAMBLE + PN532_PREAMBLE + PN532_STARTCODE2;
	wiresend(PN532_PREAMBLE);
	wiresend(PN532_PREAMBLE);
	wiresend(PN532_STARTCODE2);

	wiresend(cmdlen);
	wiresend(~cmdlen + 1);

	wiresend(PN532_HOSTTOPN532);
	checksum += PN532_HOSTTOPN532;

	#ifdef OSHU_PN532_LOWLEVEL_DEBUG
		Serial.print(" 0x"); Serial.print(PN532_PREAMBLE, HEX);
		Serial.print(" 0x"); Serial.print(PN532_PREAMBLE, HEX);
		Serial.print(" 0x"); Serial.print(PN532_STARTCODE2, HEX);
		Serial.print(" 0x"); Serial.print(cmdlen, HEX);
		Serial.print(" 0x"); Serial.print(~cmdlen + 1, HEX);
		Serial.print(" 0x"); Serial.print(PN532_HOSTTOPN532, HEX);
	#endif

	for (uint8_t i=0; i<cmdlen-1; i++) {
		wiresend(cmd[i]);
		checksum += cmd[i];
		#ifdef OSHU_PN532_LOWLEVEL_DEBUG
			Serial.print(" 0x"); Serial.print(cmd[i], HEX);
		#endif
	}

	wiresend(~checksum);
	wiresend(PN532_POSTAMBLE);

	// I2C STOP
	WIRE.endTransmission();

	#ifdef OSHU_PN532_LOWLEVEL_DEBUG
		Serial.print(" 0x"); Serial.print(~checksum, HEX);
		Serial.print(" 0x"); Serial.print(PN532_POSTAMBLE, HEX);
		Serial.println();
	#endif
} 

/**************************************************************************/
/*! 
    @brief  Waits until the PN532 is ready.

    @param  timeout   Timeout before giving up
*/
/**************************************************************************/
bool OSHU_PN532_I2C::waitUntilReady(uint16_t timeout) {
	uint16_t timer = 0;
	while(wirereadstatus() != PN532_I2C_READY) {
		if (timeout != 0) {
			timer += 10;
			if (timer > timeout) {
				return false;
			}
		}
		delay(10);
	}
	return true;
}


