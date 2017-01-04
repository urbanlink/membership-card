/**************************************************************************/
/*!
    @file     PN532.cpp
    @author   Adafruit Industries
    @license  BSD (see license.txt)

	  Driver for NXP's PN532 NFC/13.56MHz RFID Transceiver

	  This is a library for the Adafruit PN532 NFC/RFID breakout boards
	  This library works with the Adafruit NFC breakout
	  ----> https://www.adafruit.com/products/364

	  Check out the links above for our tutorials and wiring diagrams
	  These chips use SPI or I2C to communicate.

	  Adafruit invests time and resources providing this open source code,
	  please support Adafruit and open-source hardware by purchasing
	  products from Adafruit!

    @section  HISTORY

    v2.1 - Added NTAG2xx helper functions

    v2.0 - Refactored to add I2C support from Adafruit_NFCShield_I2C library.

    v1.4 - Added setPassiveActivationRetries()

    v1.2 - Added writeGPIO()
         - Added readGPIO()

    v1.1 - Changed readPassiveTargetID() to handle multiple UID sizes
         - Added the following helper functions for text display
             static void PrintHex(const byte * data, const uint32_t numBytes)
             static void PrintHexChar(const byte * pbtData, const uint32_t numBytes)
         - Added the following Mifare Classic functions:
             bool mifareclassic_IsFirstBlock (uint32_t uiBlock)
             bool mifareclassic_IsTrailerBlock (uint32_t uiBlock)
             byte mifareclassic_AuthenticateBlock (byte * uid, byte uidLen, uint32_t blockNumber, byte keyNumber, byte * keyData)
             byte mifareclassic_ReadDataBlock (byte blockNumber, byte * data)
             byte mifareclassic_WriteDataBlock (byte blockNumber, byte * data)
         - Added the following Mifare Ultalight functions:
             byte mifareultralight_ReadPage (byte page, byte * buffer)
*/
/**************************************************************************/

#include "Arduino.h"

#include <Wire.h>
#define WIRE Wire

#include "PN532.h"

byte pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
byte pn532response_firmwarevers[] = {0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03};

// Uncomment these lines to enable debug output for PN532(SPI) and/or MIFARE related code
#define PN532DEBUG
#define MIFAREDEBUG

#define PN532DEBUGPRINT Serial

#define PN532_PACKBUFFSIZ 64
byte pn532_packetbuffer[PN532_PACKBUFFSIZ];

#ifndef _BV
    #define _BV(bit) (1<<(bit))
#endif

/**************************************************************************/
/*!
    @brief  Sends a single byte via I2C

    @param  x    The byte to send
*/
/**************************************************************************/
static inline void i2c_send(byte x)
{
  #if ARDUINO >= 100
    WIRE.write((byte)x);
  #else
    WIRE.send(x);
  #endif
}

/**************************************************************************/
/*!
    @brief  Reads a single byte via I2C
*/
/**************************************************************************/
static inline byte i2c_recv(void)
{
  #if ARDUINO >= 100
    return WIRE.read();
  #else
    return WIRE.receive();
  #endif
}



/**************************************************************************/
/*!
    @brief  Instantiates a new PN532 class using I2C.


*/
/**************************************************************************/
PN532::PN532() {
  _clk          = 0;
  _miso         = 0;
  _mosi         = 0;
  _ss           = 0;
  _irq          = 0;
  _reset        = 0;
  _usingSPI     = false;
  _hardwareSPI  = false;

  Utils::Print("Initiating PN532 \r\n");
}


void PN532::InitI2C(byte irq, byte reset) {
  Utils::Print("InitI2C\r\n");

  _irq = irq;
  _reset = reset;

  pinMode(_irq, INPUT);
  pinMode(_reset, OUTPUT);
}


/**************************************************************************/
/*!
    @brief  Setups the HW

    @param  irq       Location of the IRQ pin
    @param  reset     Location of the RSTPD_N pin
*/
/**************************************************************************/
void PN532::begin() {

    // I2C initialization.
    WIRE.begin(0,2);
    Wire.setClock(100000);
	  Wire.setClockStretchLimit(2000);

    // Reset the PN532
    digitalWrite(_reset, HIGH);
    digitalWrite(_reset, LOW);
    delay(400);
    digitalWrite(_reset, HIGH);
    delay(10);  // Small delay required before taking other actions after reset.
                // See timing diagram on page 209 of the datasheet, section 12.23.

}

/**************************************************************************
    Enable / disable debug output to SerialClass
    0 = Off, 1 = high level debug, 2 = low level debug (more details)
**************************************************************************/
void PN532::SetDebugLevel(byte level) {
  mu8_DebugLevel = level;
}

/**************************************************************************/
/*!
    @brief  Prints a hexadecimal value in plain characters

    @param  data      Pointer to the byte data
    @param  numBytes  Data length in bytes
*/
/**************************************************************************/
void PN532::PrintHex(const byte * data, const uint32_t numBytes)
{
  uint32_t szPos;
  for (szPos=0; szPos < numBytes; szPos++)
  {
    PN532DEBUGPRINT.print(F("0x"));
    // Append leading 0 for small values
    if (data[szPos] <= 0xF)
      PN532DEBUGPRINT.print(F("0"));
    PN532DEBUGPRINT.print(data[szPos]&0xff, HEX);
    if ((numBytes > 1) && (szPos != numBytes - 1))
    {
      PN532DEBUGPRINT.print(F(" "));
    }
  }
  PN532DEBUGPRINT.println();
}

/**************************************************************************/
/*!
    @brief  Prints a hexadecimal value in plain characters, along with
            the char equivalents in the following format

            00 00 00 00 00 00  ......

    @param  data      Pointer to the byte data
    @param  numBytes  Data length in bytes
*/
/**************************************************************************/
void PN532::PrintHexChar(const byte * data, const uint32_t numBytes)
{
  uint32_t szPos;
  for (szPos=0; szPos < numBytes; szPos++)
  {
    // Append leading 0 for small values
    if (data[szPos] <= 0xF)
      PN532DEBUGPRINT.print(F("0"));
    PN532DEBUGPRINT.print(data[szPos], HEX);
    if ((numBytes > 1) && (szPos != numBytes - 1))
    {
      PN532DEBUGPRINT.print(F(" "));
    }
  }
  PN532DEBUGPRINT.print(F("  "));
  for (szPos=0; szPos < numBytes; szPos++)
  {
    if (data[szPos] <= 0x1F)
      PN532DEBUGPRINT.print(F("."));
    else
      PN532DEBUGPRINT.print((char)data[szPos]);
  }
  PN532DEBUGPRINT.println();
}

/**************************************************************************/
/*!
    @brief  Checks the firmware version of the PN5xx chip

    @returns  The chip's firmware version and ID
*/
/**************************************************************************/
uint32_t PN532::GetFirmwareVersion(void) {
  uint32_t response;

  pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;

  if (! SendCommandCheckAck(pn532_packetbuffer, 1)) {
    return 0;
  }

  // read data packet
  ReadData(pn532_packetbuffer, 12);

  // check some basic stuff
  if (0 != strncmp((char *)pn532_packetbuffer, (char *)pn532response_firmwarevers, 6)) {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("Firmware doesn't match!"));
    #endif
    return 0;
  }

  int offset = _usingSPI ? 6 : 7;  // Skip a response byte when using I2C to ignore extra data.
  response = pn532_packetbuffer[offset++];
  response <<= 8;
  response |= pn532_packetbuffer[offset++];
  response <<= 8;
  response |= pn532_packetbuffer[offset++];
  response <<= 8;
  response |= pn532_packetbuffer[offset++];

  return response;
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
bool PN532::SendCommandCheckAck(byte *cmd, byte cmdlen, uint16_t timeout) {
  uint16_t timer = 0;

  // write the command
  writecommand(cmd, cmdlen);

  // Wait for chip to say its ready!
  if (!WaitReady()) {
    return false;
  }

  #ifdef PN532DEBUG
    PN532DEBUGPRINT.println(F("IRQ received"));
  #endif

  // read acknowledgement
  if (!readack()) {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("No ACK frame received!"));
    #endif
    return false;
  }

  return true; // ack'd command
}

/**************************************************************************/
/*!
    Writes an 8-bit value that sets the state of the PN532's GPIO pins

    @warning This function is provided exclusively for board testing and
             is dangerous since it will throw an error if any pin other
             than the ones marked "Can be used as GPIO" are modified!  All
             pins that can not be used as GPIO should ALWAYS be left high
             (value = 1) or the system will become unstable and a HW reset
             will be required to recover the PN532.

             pinState[0]  = P30     Can be used as GPIO
             pinState[1]  = P31     Can be used as GPIO
             pinState[2]  = P32     *** RESERVED (Must be 1!) ***
             pinState[3]  = P33     Can be used as GPIO
             pinState[4]  = P34     *** RESERVED (Must be 1!) ***
             pinState[5]  = P35     Can be used as GPIO

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool PN532::WriteGPIO(byte pinstate) {
  byte errorbit;

  // Make sure pinstate does not try to toggle P32 or P34
  pinstate |= (1 << PN532_GPIO_P32) | (1 << PN532_GPIO_P34);

  // Fill command buffer
  pn532_packetbuffer[0] = PN532_COMMAND_WRITEGPIO;
  pn532_packetbuffer[1] = PN532_GPIO_VALIDATIONBIT | pinstate;  // P3 Pins
  pn532_packetbuffer[2] = 0x00;    // P7 GPIO Pins (not used ... taken by SPI)

  #ifdef PN532DEBUG
    PN532DEBUGPRINT.print(F("Writing P3 GPIO: ")); PN532DEBUGPRINT.println(pn532_packetbuffer[1], HEX);
  #endif

  // Send the WRITEGPIO command (0x0E)
  if (! SendCommandCheckAck(pn532_packetbuffer, 3))
    return 0x0;

  // Read response packet (00 FF PLEN PLENCHECKSUM D5 CMD+1(0x0F) DATACHECKSUM 00)
  ReadData(pn532_packetbuffer, 8);

  #ifdef PN532DEBUG
    PN532DEBUGPRINT.print(F("Received: "));
    PrintHex(pn532_packetbuffer, 8);
    PN532DEBUGPRINT.println();
  #endif

  int offset = _usingSPI ? 5 : 6;
  return  (pn532_packetbuffer[offset] == 0x0F);
}

/**************************************************************************/
/*!
    Reads the state of the PN532's GPIO pins

    @returns An 8-bit value containing the pin state where:

             pinState[0]  = P30
             pinState[1]  = P31
             pinState[2]  = P32
             pinState[3]  = P33
             pinState[4]  = P34
             pinState[5]  = P35
*/
/**************************************************************************/
byte PN532::ReadGPIO(void) {
  pn532_packetbuffer[0] = PN532_COMMAND_READGPIO;

  // Send the READGPIO command (0x0C)
  if (! SendCommandCheckAck(pn532_packetbuffer, 1))
    return 0x0;

  // Read response packet (00 FF PLEN PLENCHECKSUM D5 CMD+1(0x0D) P3 P7 IO1 DATACHECKSUM 00)
  ReadData(pn532_packetbuffer, 11);

  /* READGPIO response should be in the following format:

    byte            Description
    -------------   ------------------------------------------
    b0..5           Frame header and preamble (with I2C there is an extra 0x00)
    b6              P3 GPIO Pins
    b7              P7 GPIO Pins (not used ... taken by SPI)
    b8              Interface Mode Pins (not used ... bus select pins)
    b9..10          checksum */

  int p3offset = _usingSPI ? 6 : 7;

  #ifdef PN532DEBUG
    PN532DEBUGPRINT.print(F("Received: "));
    PrintHex(pn532_packetbuffer, 11);
    PN532DEBUGPRINT.println();
    PN532DEBUGPRINT.print(F("P3 GPIO: 0x")); PN532DEBUGPRINT.println(pn532_packetbuffer[p3offset],   HEX);
    PN532DEBUGPRINT.print(F("P7 GPIO: 0x")); PN532DEBUGPRINT.println(pn532_packetbuffer[p3offset+1], HEX);
    PN532DEBUGPRINT.print(F("IO GPIO: 0x")); PN532DEBUGPRINT.println(pn532_packetbuffer[p3offset+2], HEX);
    // Note: You can use the IO GPIO value to detect the serial bus being used
    switch(pn532_packetbuffer[p3offset+2])
    {
      case 0x00:    // Using UART
        PN532DEBUGPRINT.println(F("Using UART (IO = 0x00)"));
        break;
      case 0x01:    // Using I2C
        PN532DEBUGPRINT.println(F("Using I2C (IO = 0x01)"));
        break;
      case 0x02:    // Using SPI
        PN532DEBUGPRINT.println(F("Using SPI (IO = 0x02)"));
        break;
    }
  #endif

  return pn532_packetbuffer[p3offset];
}

/**************************************************************************/
/*!
    @brief  Configures the SAM (Secure Access Module)
*/
/**************************************************************************/
bool PN532::SAMConfig(void) {
  pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
  pn532_packetbuffer[1] = 0x01; // normal mode;
  pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
  pn532_packetbuffer[3] = 0x01; // use IRQ pin!

  if (! SendCommandCheckAck(pn532_packetbuffer, 4))
    return false;

  // read data packet
  ReadData(pn532_packetbuffer, 8);

  int offset = _usingSPI ? 5 : 6;
  return  (pn532_packetbuffer[offset] == 0x15);
}

/**************************************************************************/
/*!
    Sets the MxRtyPassiveActivation byte of the RFConfiguration register

    @param  maxRetries    0xFF to wait forever, 0x00..0xFE to timeout
                          after mxRetries

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool PN532::SetPassiveActivationRetries(byte maxRetries) {
  pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
  pn532_packetbuffer[1] = 5;    // Config item 5 (MaxRetries)
  pn532_packetbuffer[2] = 0xFF; // MxRtyATR (default = 0xFF)
  pn532_packetbuffer[3] = 0x01; // MxRtyPSL (default = 0x01)
  pn532_packetbuffer[4] = maxRetries;

  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Setting MxRtyPassiveActivation to ")); PN532DEBUGPRINT.print(maxRetries, DEC); PN532DEBUGPRINT.println(F(" "));
  #endif

  if (! SendCommandCheckAck(pn532_packetbuffer, 5))
    return 0x0;  // no ACK

  return 1;
}

/***** ISO14443A Commands ******/

/**************************************************************************/
/*!
    Waits for an ISO14443A target to enter the field

    @param  cardBaudRate  Baud rate of the card
    @param  uid           Pointer to the array that will be populated
                          with the card's UID (up to 7 bytes)
    @param  uidLength     Pointer to the variable that will hold the
                          length of the card's UID.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool PN532::ReadPassiveTargetID(byte* uid, byte* uidLength, eCardType* cardType) {

  // initialize return variables
  memset(uid, 0, 8);
  *uidLength = 0;
  *cardType  = CARD_Unknown;

  pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
  pn532_packetbuffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
  pn532_packetbuffer[2] = CARD_TYPE_106KB_ISO14443A;

  if (!SendCommandCheckAck(pn532_packetbuffer, 3)) {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("No card(s) read"));
    #endif
    return false;  // no cards read
  }

  // wait for a card to enter the field (only possible with I2C)
  #ifdef PN532DEBUG
    PN532DEBUGPRINT.println(F("Waiting for IRQ (indicates card presence)"));
  #endif
  if (!WaitReady()) {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("IRQ Timeout"));
    #endif
    return false;
  }

  // read the data packet with card data
  ReadData(pn532_packetbuffer, 28);
  // check some basic stuff

  /* ISO14443A card response should be in the following format:

    byte            Description
    -------------   ------------------------------------------
    b0..6           Frame header and preamble
    b7              Tags Found
    b8              Tag Number (only one used in this example)
    b9..10          SENS_RES
    b11             SEL_RES
    b12             NFCID Length
    b13..NFCIDLen   NFCID                                      */

  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Found ")); PN532DEBUGPRINT.print(pn532_packetbuffer[7], DEC); PN532DEBUGPRINT.println(F(" tags"));
  #endif
  // Return if not 1 card is found
  if (pn532_packetbuffer[7] != 1)
    return 0;

  // Get the UID length
  byte u8_IdLength = pn532_packetbuffer[12];
  if (u8_IdLength != 4 && u8_IdLength != 7) {
    Utils::Print("Card has unsupported UID length: ");
    Utils::PrintDec(u8_IdLength, LF);
    return true; // unsupported card found -> this is not an error!
  }
  // return the uid length
  *uidLength = u8_IdLength;

  // Get the UID
  for (byte i=0; i < pn532_packetbuffer[12]; i++) {
    uid[i] = pn532_packetbuffer[13+i];
  }

  uint16_t u16_ATQA = ((uint16_t)pn532_packetbuffer[9] << 8) | pn532_packetbuffer[10];
  byte     u8_SAK   = pn532_packetbuffer[11];
  if (u8_IdLength == 7 && uid[0] != 0x80 && u16_ATQA == 0x0344 && u8_SAK == 0x20) {
    *cardType = CARD_Desfire;
  }
  if (u8_IdLength == 4 && uid[0] == 0x80 && u16_ATQA == 0x0304 && u8_SAK == 0x20) {
    *cardType = CARD_DesRandom;
  }

  if (mu8_DebugLevel > 0) {
    Utils::Print("Card UID:    ");
    Utils::PrintHexBuf(uid, u8_IdLength, LF);
    char s8_Buf[80];
    sprintf(s8_Buf, "Card Type: ATQA= 0x%04X, SAK= 0x%02X", u16_ATQA, u8_SAK);

    if (*cardType == CARD_Desfire)   strcat(s8_Buf, " (Desfire Default)");
    if (*cardType == CARD_DesRandom) strcat(s8_Buf, " (Desfire RandomID)");

    Utils::Print(s8_Buf, LF);
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Exchanges an APDU with the currently inlisted peer

    @param  send            Pointer to data to send
    @param  sendLength      Length of the data to send
    @param  response        Pointer to response data
    @param  responseLength  Pointer to the response data length
*/
/**************************************************************************/
bool PN532::inDataExchange(byte * send, byte sendLength, byte * response, byte * responseLength) {
  if (sendLength > PN532_PACKBUFFSIZ-2) {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("APDU length too long for packet buffer"));
    #endif
    return false;
  }
  byte i;

  pn532_packetbuffer[0] = 0x40; // PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = _inListedTag;
  for (i=0; i<sendLength; ++i) {
    pn532_packetbuffer[i+2] = send[i];
  }

  if (!SendCommandCheckAck(pn532_packetbuffer,sendLength+2,1000)) {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("Could not send ADPU"));
    #endif
    return false;
  }

  if (!WaitReady()) {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("Response never received for ADPU..."));
    #endif
    return false;
  }

  ReadData(pn532_packetbuffer,sizeof(pn532_packetbuffer));

  if (pn532_packetbuffer[0] == 0 && pn532_packetbuffer[1] == 0 && pn532_packetbuffer[2] == 0xff) {
    byte length = pn532_packetbuffer[3];
    if (pn532_packetbuffer[4]!=(byte)(~length+1)) {
      #ifdef PN532DEBUG
        PN532DEBUGPRINT.println(F("Length check invalid"));
        PN532DEBUGPRINT.println(length,HEX);
        PN532DEBUGPRINT.println((~length)+1,HEX);
      #endif
      return false;
    }
    if (pn532_packetbuffer[5]==PN532_PN532TOHOST && pn532_packetbuffer[6]==PN532_RESPONSE_INDATAEXCHANGE) {
      if ((pn532_packetbuffer[7] & 0x3f)!=0) {
        #ifdef PN532DEBUG
          PN532DEBUGPRINT.println(F("Status code indicates an error"));
        #endif
        return false;
      }

      length -= 3;

      if (length > *responseLength) {
        length = *responseLength; // silent truncation...
      }

      for (i=0; i<length; ++i) {
        response[i] = pn532_packetbuffer[8+i];
      }
      *responseLength = length;

      return true;
    }
    else {
      PN532DEBUGPRINT.print(F("Don't know how to handle this command: "));
      PN532DEBUGPRINT.println(pn532_packetbuffer[6],HEX);
      return false;
    }
  }
  else {
    PN532DEBUGPRINT.println(F("Preamble missing"));
    return false;
  }
}

/**************************************************************************/
/*!
    @brief  'InLists' a passive target. PN532 acting as reader/initiator,
            peer acting as card/responder.
*/
/**************************************************************************/
bool PN532::inListPassiveTarget() {
  pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
  pn532_packetbuffer[1] = 1;
  pn532_packetbuffer[2] = 0;

  #ifdef PN532DEBUG
    PN532DEBUGPRINT.print(F("About to inList passive target"));
  #endif

  if (!SendCommandCheckAck(pn532_packetbuffer,3,1000)) {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("Could not send inlist message"));
    #endif
    return false;
  }

  if (!WaitReady()) {
    return false;
  }

  ReadData(pn532_packetbuffer,sizeof(pn532_packetbuffer));

  if (pn532_packetbuffer[0] == 0 && pn532_packetbuffer[1] == 0 && pn532_packetbuffer[2] == 0xff) {
    byte length = pn532_packetbuffer[3];
    if (pn532_packetbuffer[4]!=(byte)(~length+1)) {
      #ifdef PN532DEBUG
        PN532DEBUGPRINT.println(F("Length check invalid"));
        PN532DEBUGPRINT.println(length,HEX);
        PN532DEBUGPRINT.println((~length)+1,HEX);
      #endif
      return false;
    }
    if (pn532_packetbuffer[5]==PN532_PN532TOHOST && pn532_packetbuffer[6]==PN532_RESPONSE_INLISTPASSIVETARGET) {
      if (pn532_packetbuffer[7] != 1) {
        #ifdef PN532DEBUG
        PN532DEBUGPRINT.println(F("Unhandled number of targets inlisted"));
        #endif
        PN532DEBUGPRINT.println(F("Number of tags inlisted:"));
        PN532DEBUGPRINT.println(pn532_packetbuffer[7]);
        return false;
      }

      _inListedTag = pn532_packetbuffer[8];
      PN532DEBUGPRINT.print(F("Tag number: "));
      PN532DEBUGPRINT.println(_inListedTag);

      return true;
    } else {
      #ifdef PN532DEBUG
        PN532DEBUGPRINT.print(F("Unexpected response to inlist passive host"));
      #endif
      return false;
    }
  }
  else {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("Preamble missing"));
    #endif
    return false;
  }

  return true;
}


/***** Mifare Classic Functions ******/

/**************************************************************************/
/*!
      Indicates whether the specified block number is the first block
      in the sector (block 0 relative to the current sector)
*/
/**************************************************************************/
bool PN532::mifareclassic_IsFirstBlock (uint32_t uiBlock)
{
  // Test if we are in the small or big sectors
  if (uiBlock < 128)
    return ((uiBlock) % 4 == 0);
  else
    return ((uiBlock) % 16 == 0);
}

/**************************************************************************/
/*!
      Indicates whether the specified block number is the sector trailer
*/
/**************************************************************************/
bool PN532::mifareclassic_IsTrailerBlock (uint32_t uiBlock)
{
  // Test if we are in the small or big sectors
  if (uiBlock < 128)
    return ((uiBlock + 1) % 4 == 0);
  else
    return ((uiBlock + 1) % 16 == 0);
}

/**************************************************************************/
/*!
    Tries to authenticate a block of memory on a MIFARE card using the
    INDATAEXCHANGE command.  See section 7.3.8 of the PN532 User Manual
    for more information on sending MIFARE and other commands.

    @param  uid           Pointer to a byte array containing the card UID
    @param  uidLen        The length (in bytes) of the card's UID (Should
                          be 4 for MIFARE Classic)
    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  keyNumber     Which key type to use during authentication
                          (0 = MIFARE_CMD_AUTH_A, 1 = MIFARE_CMD_AUTH_B)
    @param  keyData       Pointer to a byte array containing the 6 byte
                          key value

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
byte PN532::mifareclassic_AuthenticateBlock (byte * uid, byte uidLen, uint32_t blockNumber, byte keyNumber, byte * keyData)
{
  byte len;
  byte i;

  // Hang on to the key and uid data
  memcpy (_key, keyData, 6);
  memcpy (_uid, uid, uidLen);
  _uidLen = uidLen;

  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Trying to authenticate card "));
    PN532::PrintHex(_uid, _uidLen);
    PN532DEBUGPRINT.print(F("Using authentication KEY "));PN532DEBUGPRINT.print(keyNumber ? 'B' : 'A');PN532DEBUGPRINT.print(F(": "));
    PN532::PrintHex(_key, 6);
  #endif

  // Prepare the authentication command //
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;   /* Data Exchange Header */
  pn532_packetbuffer[1] = 1;                              /* Max card numbers */
  pn532_packetbuffer[2] = (keyNumber) ? MIFARE_CMD_AUTH_B : MIFARE_CMD_AUTH_A;
  pn532_packetbuffer[3] = blockNumber;                    /* Block Number (1K = 0..63, 4K = 0..255 */
  memcpy (pn532_packetbuffer+4, _key, 6);
  for (i = 0; i < _uidLen; i++)
  {
    pn532_packetbuffer[10+i] = _uid[i];                /* 4 byte card ID */
  }

  if (! SendCommandCheckAck(pn532_packetbuffer, 10+_uidLen))
    return 0;

  // Read the response packet
  ReadData(pn532_packetbuffer, 12);

  // check if the response is valid and we are authenticated???
  // for an auth success it should be bytes 5-7: 0xD5 0x41 0x00
  // Mifare auth error is technically byte 7: 0x14 but anything other and 0x00 is not good
  if (pn532_packetbuffer[7] != 0x00)
  {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.print(F("Authentification failed: "));
      PN532::PrintHexChar(pn532_packetbuffer, 12);
    #endif
    return 0;
  }

  return 1;
}

/**************************************************************************/
/*!
    Tries to read an entire 16-byte data block at the specified block
    address.

    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  data          Pointer to the byte array that will hold the
                          retrieved data (if any)

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
byte PN532::mifareclassic_ReadDataBlock (byte blockNumber, byte * data)
{
  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Trying to read 16 bytes from block "));PN532DEBUGPRINT.println(blockNumber);
  #endif

  /* Prepare the command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                      /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_READ;        /* Mifare Read command = 0x30 */
  pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */

  /* Send the command */
  if (! SendCommandCheckAck(pn532_packetbuffer, 4))
  {
    #ifdef MIFAREDEBUG
      PN532DEBUGPRINT.println(F("Failed to receive ACK for read command"));
    #endif
    return 0;
  }

  /* Read the response packet */
  ReadData(pn532_packetbuffer, 26);

  /* If byte 8 isn't 0x00 we probably have an error */
  if (pn532_packetbuffer[7] != 0x00)
  {
    #ifdef MIFAREDEBUG
      PN532DEBUGPRINT.println(F("Unexpected response"));
      PN532::PrintHexChar(pn532_packetbuffer, 26);
    #endif
    return 0;
  }

  /* Copy the 16 data bytes to the output buffer        */
  /* Block content starts at byte 9 of a valid response */
  memcpy (data, pn532_packetbuffer+8, 16);

  /* Display data for debug if requested */
  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Block "));
    PN532DEBUGPRINT.println(blockNumber);
    PN532::PrintHexChar(data, 16);
  #endif

  return 1;
}

/**************************************************************************/
/*!
    Tries to write an entire 16-byte data block at the specified block
    address.

    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  data          The byte array that contains the data to write.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
byte PN532::mifareclassic_WriteDataBlock (byte blockNumber, byte * data)
{
  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Trying to write 16 bytes to block "));PN532DEBUGPRINT.println(blockNumber);
  #endif

  /* Prepare the first command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                      /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_WRITE;       /* Mifare Write command = 0xA0 */
  pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */
  memcpy (pn532_packetbuffer+4, data, 16);          /* Data Payload */

  /* Send the command */
  if (! SendCommandCheckAck(pn532_packetbuffer, 20))
  {
    #ifdef MIFAREDEBUG
      PN532DEBUGPRINT.println(F("Failed to receive ACK for write command"));
    #endif
    return 0;
  }
  delay(10);

  /* Read the response packet */
  ReadData(pn532_packetbuffer, 26);

  return 1;
}

/**************************************************************************/
/*!
    Formats a Mifare Classic card to store NDEF Records

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
byte PN532::mifareclassic_FormatNDEF (void)
{
  byte sectorbuffer1[16] = {0x14, 0x01, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
  byte sectorbuffer2[16] = {0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
  byte sectorbuffer3[16] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0x78, 0x77, 0x88, 0xC1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  // Note 0xA0 0xA1 0xA2 0xA3 0xA4 0xA5 must be used for key A
  // for the MAD sector in NDEF records (sector 0)

  // Write block 1 and 2 to the card
  if (!(mifareclassic_WriteDataBlock (1, sectorbuffer1)))
    return 0;
  if (!(mifareclassic_WriteDataBlock (2, sectorbuffer2)))
    return 0;
  // Write key A and access rights card
  if (!(mifareclassic_WriteDataBlock (3, sectorbuffer3)))
    return 0;

  // Seems that everything was OK (?!)
  return 1;
}

/**************************************************************************/
/*!
    Writes an NDEF URI Record to the specified sector (1..15)

    Note that this function assumes that the Mifare Classic card is
    already formatted to work as an "NFC Forum Tag" and uses a MAD1
    file system.  You can use the NXP TagWriter app on Android to
    properly format cards for this.

    @param  sectorNumber  The sector that the URI record should be written
                          to (can be 1..15 for a 1K card)
    @param  uriIdentifier The uri identifier code (0 = none, 0x01 =
                          "http://www.", etc.)
    @param  url           The uri text to write (max 38 characters).

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
byte PN532::mifareclassic_WriteNDEFURI (byte sectorNumber, byte uriIdentifier, const char * url)
{
  // Figure out how long the string is
  byte len = strlen(url);

  // Make sure we're within a 1K limit for the sector number
  if ((sectorNumber < 1) || (sectorNumber > 15))
    return 0;

  // Make sure the URI payload is between 1 and 38 chars
  if ((len < 1) || (len > 38))
    return 0;

  // Note 0xD3 0xF7 0xD3 0xF7 0xD3 0xF7 must be used for key A
  // in NDEF records

  // Setup the sector buffer (w/pre-formatted TLV wrapper and NDEF message)
  byte sectorbuffer1[16] = {0x00, 0x00, 0x03, len+5, 0xD1, 0x01, len+1, 0x55, uriIdentifier, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte sectorbuffer2[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte sectorbuffer3[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte sectorbuffer4[16] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7, 0x7F, 0x07, 0x88, 0x40, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  if (len <= 6)
  {
    // Unlikely we'll get a url this short, but why not ...
    memcpy (sectorbuffer1+9, url, len);
    sectorbuffer1[len+9] = 0xFE;
  }
  else if (len == 7)
  {
    // 0xFE needs to be wrapped around to next block
    memcpy (sectorbuffer1+9, url, len);
    sectorbuffer2[0] = 0xFE;
  }
  else if ((len > 7) && (len <= 22))
  {
    // Url fits in two blocks
    memcpy (sectorbuffer1+9, url, 7);
    memcpy (sectorbuffer2, url+7, len-7);
    sectorbuffer2[len-7] = 0xFE;
  }
  else if (len == 23)
  {
    // 0xFE needs to be wrapped around to final block
    memcpy (sectorbuffer1+9, url, 7);
    memcpy (sectorbuffer2, url+7, len-7);
    sectorbuffer3[0] = 0xFE;
  }
  else
  {
    // Url fits in three blocks
    memcpy (sectorbuffer1+9, url, 7);
    memcpy (sectorbuffer2, url+7, 16);
    memcpy (sectorbuffer3, url+23, len-24);
    sectorbuffer3[len-22] = 0xFE;
  }

  // Now write all three blocks back to the card
  if (!(mifareclassic_WriteDataBlock (sectorNumber*4, sectorbuffer1)))
    return 0;
  if (!(mifareclassic_WriteDataBlock ((sectorNumber*4)+1, sectorbuffer2)))
    return 0;
  if (!(mifareclassic_WriteDataBlock ((sectorNumber*4)+2, sectorbuffer3)))
    return 0;
  if (!(mifareclassic_WriteDataBlock ((sectorNumber*4)+3, sectorbuffer4)))
    return 0;

  // Seems that everything was OK (?!)
  return 1;
}

/***** Mifare Ultralight Functions ******/

/**************************************************************************/
/*!
    Tries to read an entire 4-byte page at the specified address.

    @param  page        The page number (0..63 in most cases)
    @param  buffer      Pointer to the byte array that will hold the
                        retrieved data (if any)
*/
/**************************************************************************/
byte PN532::mifareultralight_ReadPage (byte page, byte * buffer)
{
  if (page >= 64)
  {
    #ifdef MIFAREDEBUG
      PN532DEBUGPRINT.println(F("Page value out of range"));
    #endif
    return 0;
  }

  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Reading page "));PN532DEBUGPRINT.println(page);
  #endif

  /* Prepare the command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                   /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_READ;     /* Mifare Read command = 0x30 */
  pn532_packetbuffer[3] = page;                /* Page Number (0..63 in most cases) */

  /* Send the command */
  if (! SendCommandCheckAck(pn532_packetbuffer, 4))
  {
    #ifdef MIFAREDEBUG
      PN532DEBUGPRINT.println(F("Failed to receive ACK for write command"));
    #endif
    return 0;
  }

  /* Read the response packet */
  ReadData(pn532_packetbuffer, 26);
  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.println(F("Received: "));
    PN532::PrintHexChar(pn532_packetbuffer, 26);
  #endif

  /* If byte 8 isn't 0x00 we probably have an error */
  if (pn532_packetbuffer[7] == 0x00)
  {
    /* Copy the 4 data bytes to the output buffer         */
    /* Block content starts at byte 9 of a valid response */
    /* Note that the command actually reads 16 byte or 4  */
    /* pages at a time ... we simply discard the last 12  */
    /* bytes                                              */
    memcpy (buffer, pn532_packetbuffer+8, 4);
  }
  else
  {
    #ifdef MIFAREDEBUG
      PN532DEBUGPRINT.println(F("Unexpected response reading block: "));
      PN532::PrintHexChar(pn532_packetbuffer, 26);
    #endif
    return 0;
  }

  /* Display data for debug if requested */
  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Page "));PN532DEBUGPRINT.print(page);PN532DEBUGPRINT.println(F(":"));
    PN532::PrintHexChar(buffer, 4);
  #endif

  // Return OK signal
  return 1;
}

/**************************************************************************/
/*!
    Tries to write an entire 4-byte page at the specified block
    address.

    @param  page          The page number to write.  (0..63 for most cases)
    @param  data          The byte array that contains the data to write.
                          Should be exactly 4 bytes long.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
byte PN532::mifareultralight_WritePage (byte page, byte * data)
{

  if (page >= 64)
  {
    #ifdef MIFAREDEBUG
      PN532DEBUGPRINT.println(F("Page value out of range"));
    #endif
    // Return Failed Signal
    return 0;
  }

  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Trying to write 4 byte page"));PN532DEBUGPRINT.println(page);
  #endif

  /* Prepare the first command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                      /* Card number */
  pn532_packetbuffer[2] = MIFARE_ULTRALIGHT_CMD_WRITE;       /* Mifare Ultralight Write command = 0xA2 */
  pn532_packetbuffer[3] = page;            /* Page Number (0..63 for most cases) */
  memcpy (pn532_packetbuffer+4, data, 4);          /* Data Payload */

  /* Send the command */
  if (! SendCommandCheckAck(pn532_packetbuffer, 8))
  {
    #ifdef MIFAREDEBUG
      PN532DEBUGPRINT.println(F("Failed to receive ACK for write command"));
    #endif

    // Return Failed Signal
    return 0;
  }
  delay(10);

  /* Read the response packet */
  ReadData(pn532_packetbuffer, 26);

  // Return OK Signal
  return 1;
}


/***** NTAG2xx Functions ******/

/**************************************************************************/
/*!
    Tries to read an entire 4-byte page at the specified address.

    @param  page        The page number (0..63 in most cases)
    @param  buffer      Pointer to the byte array that will hold the
                        retrieved data (if any)
*/
/**************************************************************************/
byte PN532::ntag2xx_ReadPage (byte page, byte * buffer)
{
  // TAG Type       PAGES   USER START    USER STOP
  // --------       -----   ----------    ---------
  // NTAG 203       42      4             39
  // NTAG 213       45      4             39
  // NTAG 215       135     4             129
  // NTAG 216       231     4             225

  if (page >= 231)
  {
    #ifdef MIFAREDEBUG
      PN532DEBUGPRINT.println(F("Page value out of range"));
    #endif
    return 0;
  }

  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Reading page "));PN532DEBUGPRINT.println(page);
  #endif

  /* Prepare the command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                   /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_READ;     /* Mifare Read command = 0x30 */
  pn532_packetbuffer[3] = page;                /* Page Number (0..63 in most cases) */

  /* Send the command */
  if (! SendCommandCheckAck(pn532_packetbuffer, 4))
  {
    #ifdef MIFAREDEBUG
      PN532DEBUGPRINT.println(F("Failed to receive ACK for write command"));
    #endif
    return 0;
  }

  /* Read the response packet */
  ReadData(pn532_packetbuffer, 26);
  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.println(F("Received: "));
    PN532::PrintHexChar(pn532_packetbuffer, 26);
  #endif

  /* If byte 8 isn't 0x00 we probably have an error */
  if (pn532_packetbuffer[7] == 0x00)
  {
    /* Copy the 4 data bytes to the output buffer         */
    /* Block content starts at byte 9 of a valid response */
    /* Note that the command actually reads 16 byte or 4  */
    /* pages at a time ... we simply discard the last 12  */
    /* bytes                                              */
    memcpy (buffer, pn532_packetbuffer+8, 4);
  }
  else
  {
    #ifdef MIFAREDEBUG
      PN532DEBUGPRINT.println(F("Unexpected response reading block: "));
      PN532::PrintHexChar(pn532_packetbuffer, 26);
    #endif
    return 0;
  }

  /* Display data for debug if requested */
  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Page "));PN532DEBUGPRINT.print(page);PN532DEBUGPRINT.println(F(":"));
    PN532::PrintHexChar(buffer, 4);
  #endif

  // Return OK signal
  return 1;
}

/**************************************************************************/
/*!
    Tries to write an entire 4-byte page at the specified block
    address.

    @param  page          The page number to write.  (0..63 for most cases)
    @param  data          The byte array that contains the data to write.
                          Should be exactly 4 bytes long.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
byte PN532::ntag2xx_WritePage (byte page, byte * data)
{
  // TAG Type       PAGES   USER START    USER STOP
  // --------       -----   ----------    ---------
  // NTAG 203       42      4             39
  // NTAG 213       45      4             39
  // NTAG 215       135     4             129
  // NTAG 216       231     4             225

  if ((page < 4) || (page > 225))
  {
    #ifdef MIFAREDEBUG
      PN532DEBUGPRINT.println(F("Page value out of range"));
    #endif
    // Return Failed Signal
    return 0;
  }

  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Trying to write 4 byte page"));PN532DEBUGPRINT.println(page);
  #endif

  /* Prepare the first command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                              /* Card number */
  pn532_packetbuffer[2] = MIFARE_ULTRALIGHT_CMD_WRITE;    /* Mifare Ultralight Write command = 0xA2 */
  pn532_packetbuffer[3] = page;                           /* Page Number (0..63 for most cases) */
  memcpy (pn532_packetbuffer+4, data, 4);                 /* Data Payload */

  /* Send the command */
  if (! SendCommandCheckAck(pn532_packetbuffer, 8))
  {
    #ifdef MIFAREDEBUG
      PN532DEBUGPRINT.println(F("Failed to receive ACK for write command"));
    #endif

    // Return Failed Signal
    return 0;
  }
  delay(10);

  /* Read the response packet */
  ReadData(pn532_packetbuffer, 26);

  // Return OK Signal
  return 1;
}

/**************************************************************************/
/*!
    Writes an NDEF URI Record starting at the specified page (4..nn)

    Note that this function assumes that the NTAG2xx card is
    already formatted to work as an "NFC Forum Tag".

    @param  uriIdentifier The uri identifier code (0 = none, 0x01 =
                          "http://www.", etc.)
    @param  url           The uri text to write (null-terminated string).
    @param  dataLen       The size of the data area for overflow checks.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
byte PN532::ntag2xx_WriteNDEFURI (byte uriIdentifier, char * url, byte dataLen)
{
  byte pageBuffer[4] = { 0, 0, 0, 0 };

  // Remove NDEF record overhead from the URI data (pageHeader below)
  byte wrapperSize = 12;

  // Figure out how long the string is
  byte len = strlen(url);

  // Make sure the URI payload will fit in dataLen (include 0xFE trailer)
  if ((len < 1) || (len+1 > (dataLen-wrapperSize)))
    return 0;

  // Setup the record header
  // See NFCForum-TS-Type-2-Tag_1.1.pdf for details
  byte pageHeader[12] =
  {
    /* NDEF Lock Control TLV (must be first and always present) */
    0x01,         /* Tag Field (0x01 = Lock Control TLV) */
    0x03,         /* Payload Length (always 3) */
    0xA0,         /* The position inside the tag of the lock bytes (upper 4 = page address, lower 4 = byte offset) */
    0x10,         /* Size in bits of the lock area */
    0x44,         /* Size in bytes of a page and the number of bytes each lock bit can lock (4 bit + 4 bits) */
    /* NDEF Message TLV - URI Record */
    0x03,         /* Tag Field (0x03 = NDEF Message) */
    len+5,        /* Payload Length (not including 0xFE trailer) */
    0xD1,         /* NDEF Record Header (TNF=0x1:Well known record + SR + ME + MB) */
    0x01,         /* Type Length for the record type indicator */
    len+1,        /* Payload len */
    0x55,         /* Record Type Indicator (0x55 or 'U' = URI Record) */
    uriIdentifier /* URI Prefix (ex. 0x01 = "http://www.") */
  };

  // Write 12 byte header (three pages of data starting at page 4)
  memcpy (pageBuffer, pageHeader, 4);
  if (!(ntag2xx_WritePage (4, pageBuffer)))
    return 0;
  memcpy (pageBuffer, pageHeader+4, 4);
  if (!(ntag2xx_WritePage (5, pageBuffer)))
    return 0;
  memcpy (pageBuffer, pageHeader+8, 4);
  if (!(ntag2xx_WritePage (6, pageBuffer)))
    return 0;

  // Write URI (starting at page 7)
  byte currentPage = 7;
  char * urlcopy = url;
  while(len)
  {
    if (len < 4)
    {
      memset(pageBuffer, 0, 4);
      memcpy(pageBuffer, urlcopy, len);
      pageBuffer[len] = 0xFE; // NDEF record footer
      if (!(ntag2xx_WritePage (currentPage, pageBuffer)))
        return 0;
      // DONE!
      return 1;
    }
    else if (len == 4)
    {
      memcpy(pageBuffer, urlcopy, len);
      if (!(ntag2xx_WritePage (currentPage, pageBuffer)))
        return 0;
      memset(pageBuffer, 0, 4);
      pageBuffer[0] = 0xFE; // NDEF record footer
      currentPage++;
      if (!(ntag2xx_WritePage (currentPage, pageBuffer)))
        return 0;
      // DONE!
      return 1;
    }
    else
    {
      // More than one page of data left
      memcpy(pageBuffer, urlcopy, 4);
      if (!(ntag2xx_WritePage (currentPage, pageBuffer)))
        return 0;
      currentPage++;
      urlcopy+=4;
      len-=4;
    }
  }

  // Seems that everything was OK (?!)
  return 1;
}


/************** high level communication functions (handles both I2C and SPI) */


/**************************************************************************/
/*!
    @brief  Tries to read the SPI or I2C ACK signal
*/
/**************************************************************************/
bool PN532::readack() {
  byte ackbuff[6];

  ReadData(ackbuff, 6);

  return (0 == strncmp((char *)ackbuff, (char *)pn532ack, 6));
}


/**************************************************************************/
/*!
    @brief  Return true if the PN532 is ready with a response.
*/
/**************************************************************************/
bool PN532::IsReady() {

  // I2C check if status is ready by IRQ line being pulled low.
  byte x = digitalRead(_irq);

  return x == 0;
}

/**************************************************************************/
/*!
    @brief  Waits until the PN532 is ready.

    @param  timeout   Timeout before giving up
*/
/**************************************************************************/
bool PN532::WaitReady() {
  uint16_t timer = 0;
  while (!IsReady()) {
    if (timer >= PN532_TIMEOUT) {
      Utils::Print("WaitReady() -> TIMEOUT\r\n");
      return false;
    }
    Utils::DelayMilli(10);
    timer += 10;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Reads n bytes of data from the PN532 via SPI or I2C.

    @param  buff      Pointer to the buffer where data will be written
    @param  n         Number of bytes to be read
*/
/**************************************************************************/
byte PN532::ReadData(byte* buff, byte n) {

    // I2C write.
    uint16_t timer = 0;

    delay(2);

    #ifdef PN532DEBUG
      PN532DEBUGPRINT.print(F("Reading: "));
    #endif
    // Start read (n+1 to take into account leading 0x01 with I2C)
    WIRE.requestFrom((byte)PN532_I2C_ADDRESS, (byte)(n+2));
    // Discard the leading 0x01
    i2c_recv();
    for (byte i=0; i<n; i++) {
      delay(1);
      buff[i] = i2c_recv();
      #ifdef PN532DEBUG
        PN532DEBUGPRINT.print(F(" 0x"));
        PN532DEBUGPRINT.print(buff[i], HEX);
      #endif
    }
    // Discard trailing 0x00 0x00
    // i2c_recv();

    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println();
    #endif

  return 1;
}

/**************************************************************************/
/*!
    @brief  Writes a command to the PN532, automatically inserting the
            preamble and required frame details (checksum, len, etc.)

    @param  cmd       Pointer to the command buffer
    @param  cmdlen    Command length in bytes
*/
/**************************************************************************/
void PN532::writecommand(byte* cmd, byte cmdlen) {
    // I2C command write.
    byte checksum;

    cmdlen++;

    #ifdef PN532DEBUG
      PN532DEBUGPRINT.print(F("\nSending: "));
    #endif

    delay(2);     // or whatever the delay is for waking up the board

    // I2C START
    WIRE.beginTransmission(PN532_I2C_ADDRESS);
    checksum = PN532_PREAMBLE + PN532_PREAMBLE + PN532_STARTCODE2;
    i2c_send(PN532_PREAMBLE);
    i2c_send(PN532_PREAMBLE);
    i2c_send(PN532_STARTCODE2);

    i2c_send(cmdlen);
    i2c_send(~cmdlen + 1);

    i2c_send(PN532_HOSTTOPN532);
    checksum += PN532_HOSTTOPN532;

    #ifdef PN532DEBUG
      PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print(PN532_PREAMBLE, HEX);
      PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print(PN532_PREAMBLE, HEX);
      PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print(PN532_STARTCODE2, HEX);
      PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print(cmdlen, HEX);
      PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print(~cmdlen + 1, HEX);
      PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print(PN532_HOSTTOPN532, HEX);
    #endif

    for (byte i=0; i<cmdlen-1; i++) {
      i2c_send(cmd[i]);
      checksum += cmd[i];
      #ifdef PN532DEBUG
        PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print(cmd[i], HEX);
      #endif
    }

    i2c_send(~checksum);
    i2c_send(PN532_POSTAMBLE);

    // I2C STOP
    WIRE.endTransmission();

    #ifdef PN532DEBUG
      PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print(~checksum, HEX);
      PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print(PN532_POSTAMBLE, HEX);
      PN532DEBUGPRINT.println();
    #endif
}


/**************************************************************************
    Turns the RF field off.
    When the field is on, the PN532 consumes approx 110 mA
    When the field is off, the PN532 consumes approx 18 mA
    The RF field is turned on again by ReadPassiveTargetID().
**************************************************************************/
bool PN532::SwitchOffRfField() {
  if (mu8_DebugLevel > 0) Utils::Print("\r\n*** SwitchOffRfField()\r\n");

  mu8_PacketBuffer[0] = PN532_COMMAND_RFCONFIGURATION;
  mu8_PacketBuffer[1] = 1; // Config item 1 (RF Field)
  mu8_PacketBuffer[2] = 0; // Field Off

  if (!SendCommandCheckAck(mu8_PacketBuffer, 3))
      return false;

  byte len = ReadData(mu8_PacketBuffer, 9);
  if (len != 2 || mu8_PacketBuffer[1] != PN532_COMMAND_RFCONFIGURATION + 1)
  {
      Utils::Print("SwitchOffRfField failed\r\n");
      return false;
  }
  return true;
}

/**************************************************************************
    This function is private
    It checks the status byte that is returned by some commands.
    See chapter 7.1 in the manual.
    u8_Status = the status byte
**************************************************************************/
bool PN532::CheckPN532Status(byte u8_Status)
{
    // Bits 0...5 contain the error code.
    u8_Status &= 0x3F;

    if (u8_Status == 0)
        return true;

    char s8_Buf[50];
    sprintf(s8_Buf, "PN532 Error 0x%02X: ", u8_Status);
    Utils::Print(s8_Buf);

    switch (u8_Status)
    {
        case 0x01:
            Utils::Print("Timeout\r\n");
            return false;
        case 0x02:
            Utils::Print("CRC error\r\n");
            return false;
        case 0x03:
            Utils::Print("Parity error\r\n");
            return false;
        case 0x04:
            Utils::Print("Wrong bit count during anti-collision\r\n");
            return false;
        case 0x05:
            Utils::Print("Framing error\r\n");
            return false;
        case 0x06:
            Utils::Print("Abnormal bit collision\r\n");
            return false;
        case 0x07:
            Utils::Print("Insufficient communication buffer\r\n");
            return false;
        case 0x09:
            Utils::Print("RF buffer overflow\r\n");
            return false;
        case 0x0A:
            Utils::Print("RF field has not been switched on\r\n");
            return false;
        case 0x0B:
            Utils::Print("RF protocol error\r\n");
            return false;
        case 0x0D:
            Utils::Print("Overheating\r\n");
            return false;
        case 0x0E:
            Utils::Print("Internal buffer overflow\r\n");
            return false;
        case 0x10:
            Utils::Print("Invalid parameter\r\n");
            return false;
        case 0x12:
            Utils::Print("Command not supported\r\n");
            return false;
        case 0x13:
            Utils::Print("Wrong data format\r\n");
            return false;
        case 0x14:
            Utils::Print("Authentication error\r\n");
            return false;
        case 0x23:
            Utils::Print("Wrong UID check byte\r\n");
            return false;
        case 0x25:
            Utils::Print("Invalid device state\r\n");
            return false;
        case 0x26:
            Utils::Print("Operation not allowed\r\n");
            return false;
        case 0x27:
            Utils::Print("Command not acceptable\r\n");
            return false;
        case 0x29:
            Utils::Print("Target has been released\r\n");
            return false;
        case 0x2A:
            Utils::Print("Card has been exchanged\r\n");
            return false;
        case 0x2B:
            Utils::Print("Card has disappeared\r\n");
            return false;
        case 0x2C:
            Utils::Print("NFCID3 initiator/target mismatch\r\n");
            return false;
        case 0x2D:
            Utils::Print("Over-current\r\n");
            return false;
        case 0x2E:
            Utils::Print("NAD msssing\r\n");
            return false;
        default:
            Utils::Print("Undocumented error\r\n");
            return false;
    }
}
