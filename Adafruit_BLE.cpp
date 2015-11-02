/**************************************************************************/
/*!
    @file     Adafruit_BLE.c
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2014, Adafruit Industries (adafruit.com)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include "Adafruit_BLE.h"

#ifndef min
  #define min(a,b) ((a) < (b) ? (a) : (b))
#endif

enum {
  EVENT_SYSTEM_CONNECT     = 0,
  EVENT_SYSTEM_DISCONNECT  = 1,
  EVENT_SYSTEM_BLE_UART_RX = 8,
  EVENT_SYSTEM_BLE_MIDI_RX = 10,
};

void Adafruit_BLE::install_callback(bool enable, uint8_t system_id, uint8_t gatts_id)
{
  bool v = _verbose;
  _verbose = true;

  uint8_t current_mode = _mode;

  // switch mode if necessary to execute command
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_COMMAND);

  print( enable ?  F("AT+EVENTENABLE=0x") : F("AT+EVENTDISABLE=0x") );
  println( bit(system_id), HEX );
  waitForOK();

  // switch back if necessary
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_DATA);

  _verbose = v;
}
/******************************************************************************/
/*!
    @brief  Constructor
*/
/******************************************************************************/
Adafruit_BLE::Adafruit_BLE(void)
{
  _verbose = false;
  _mode    = BLUEFRUIT_MODE_COMMAND;
  _timeout = BLE_DEFAULT_TIMEOUT;

  _disconnect_callback  = NULL;
  _connect_callback     = NULL;
  _ble_uart_rx_callback = NULL;
  _ble_midi_rx_callback = NULL;
}

/******************************************************************************/
/*!
    @brief  Performs a system reset using AT command
*/
/******************************************************************************/
bool Adafruit_BLE::reset(void)
{
  bool isOK;
  // println();
  for (uint8_t t=0; t < 5; t++) {
    isOK = sendCommandCheckOK(F("ATZ"));

    if (isOK) break;
  }

  if (! isOK) {
    // ok we're going to get desperate
    delay(50);
    setMode(BLUEFRUIT_MODE_COMMAND);
    delay(50);
    
    for (uint8_t t=0; t < 5; t++) {
      isOK = sendCommandCheckOK(F("ATZ"));
      
      if (isOK) break;
    }

    if (!isOK) return false;
  }

  // Bluefruit need 1 second to reboot
  delay(1000);

  // flush all left over
  flush();

  return isOK;
}

/******************************************************************************/
/*!
    @brief  Performs a factory reset
*/
/******************************************************************************/
bool Adafruit_BLE::factoryReset(void)
{
  println( F("AT+FACTORYRESET") );
  bool isOK = waitForOK();

  // Bluefruit need 1 second to reboot
  delay(1000);

  // flush all left over
  flush();

  return isOK;
}

/******************************************************************************/
/*!
    @brief  Enable or disable AT Command echo from Bluefruit

    @parma[in] enable
               true to enable (default), false to disable
*/
/******************************************************************************/
bool Adafruit_BLE::echo(bool enable)
{
  if (enable)
  {
    return sendCommandCheckOK( F("ATE=1") );
  }else
  {
    return sendCommandCheckOK( F("ATE=0") );
  }
}

/******************************************************************************/
/*!
    @brief  Check connection state, returns true is connected!
*/
/******************************************************************************/
bool Adafruit_BLE::isConnected(void)
{
  int32_t connected = 0;
  sendCommandWithIntReply(F("AT+GAPGETCONN"), &connected);

  return connected;
}


/******************************************************************************/
/*!
    @brief  Print Bluefruit's information retrieved by ATI command
*/
/******************************************************************************/
void Adafruit_BLE::info(void)
{
  uint8_t current_mode = _mode;

  bool v = _verbose;
  _verbose = false;

  SerialDebug.println(F("----------------"));

  // switch mode if necessary to execute command
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_COMMAND);

  println(F("ATI"));

  while ( readline() ) {
    if ( !strcmp(buffer, "OK") || !strcmp(buffer, "ERROR")  ) break;
    SerialDebug.println(buffer);
  }

  // switch back if necessary
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_DATA);

  SerialDebug.println(F("----------------"));

  _verbose = v;
}

/**************************************************************************/
/*!
    @brief  Checks if firmware is equal or later than specified version
*/
/**************************************************************************/
bool Adafruit_BLE::isVersionAtLeast(char * versionString)
{
  uint8_t current_mode = _mode;

  // switch mode if necessary to execute command
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_COMMAND);

  // requesting version number
  println(F("ATI=4"));

  readline();
  bool result = ( strcmp(buffer, versionString) >= 0 );
  waitForOK();

  // switch back if necessary
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_DATA);

  return result;
}

/******************************************************************************/
/*!
    @brief  Send a command from a flash string, and parse an int reply
*/
/******************************************************************************/
bool Adafruit_BLE::sendCommandWithIntReply(const __FlashStringHelper *cmd, int32_t *reply)
{
  bool result;
  uint8_t current_mode = _mode;

  // switch mode if necessary to execute command
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_COMMAND);

  println(cmd);                   // send command
  if (_verbose) {
    SerialDebug.print( F("\n<- ") );
  }
  (*reply) = readline_parseInt(); // parse integer response
  result = waitForOK();

  // switch back if necessary
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_DATA);

  return result;
}

/******************************************************************************/
/*!
    @brief  Send a command from a SRAM string, and parse an int reply
*/
/******************************************************************************/
bool Adafruit_BLE::sendCommandWithIntReply(const char cmd[], int32_t *reply) {
  bool result;
  uint8_t current_mode = _mode;

  // switch mode if necessary to execute command
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_COMMAND);

  println(cmd);                   // send command
  if (_verbose) {
    SerialDebug.print( F("\n<- ") );
  }
  (*reply) = readline_parseInt(); // parse integer response
  result = waitForOK();

  // switch back if necessary
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_DATA);

  return result;
}


/******************************************************************************/
/*!
    @brief  Send a command from a flash string, and parse an int reply
*/
/******************************************************************************/
bool Adafruit_BLE::sendCommandCheckOK(const __FlashStringHelper *cmd)
{
  bool result;
  uint8_t current_mode = _mode;

  // switch mode if necessary to execute command
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_COMMAND);

  println(cmd);       // send command
  result = waitForOK();

  // switch back if necessary
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_DATA);

  return result;
}

/******************************************************************************/
/*!
    @brief  Send a command from a SRAM string, and parse an int reply
*/
/******************************************************************************/
bool Adafruit_BLE::sendCommandCheckOK(const char cmd[])
{
  bool result;
  uint8_t current_mode = _mode;

  // switch mode if necessary to execute command
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_COMMAND);

  println(cmd);       // send command
  result = waitForOK();

  // switch back if necessary
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_DATA);

  return result;
}

/******************************************************************************/
/*!
    @brief  Read the whole response and check if it ended up with OK.
    @return true if response is ended with "OK". Otherwise it could be "ERROR"
*/
/******************************************************************************/
bool Adafruit_BLE::waitForOK(void)
{
  if (_verbose) {
    SerialDebug.print( F("\n<- ") );
  }

  while ( readline() ) {
    //SerialDebug.println(buffer);
    if ( strcmp(buffer, "OK") == 0 ) return true;
    if ( strcmp(buffer, "ERROR") == 0 ) return false;
  }
  return false;
}

/******************************************************************************/
/*!
    @brief  Get a line of response data (see \ref readline) and try to interpret
            it to an integer number. If the number is prefix with '0x', it will
            be interpreted as hex number. This function also drop the rest of
            data to the end of the line.
*/
/******************************************************************************/
int32_t Adafruit_BLE::readline_parseInt(void)
{
  uint16_t len = readline();
  if (len == 0) return 0;

  // also parsed hex number e.g 0xADAF
  int32_t val = strtol(buffer, NULL, 0);

  return val;
}

/******************************************************************************/
/*!
    @brief  Get a line of response data into provided buffer.

    @param[in] buf
               Provided buffer
    @param[in] bufsize
               buffer size
*/
/******************************************************************************/
uint16_t Adafruit_BLE::readline(char * buf, uint16_t bufsize)
{
  uint16_t len = bufsize;
  uint16_t rd  = 0;

  while ( (len > 0) && ((rd = readline()) == BLE_BUFSIZE)  )
  {
    uint16_t n = min(len, rd);
    memcpy(buf, buffer, n);

    buf += n;
    len -= n;
  }

//  buf[bufsize - len] = 0; // null terminator

  return bufsize - len;
}

/******************************************************************************/
/*!
    @brief  Get (multiple) lines of response data into internal buffer.

    @param[in] timeout
               Timeout for each read() operation
    @param[in] multiline
               Read multiple lines

    @return    The number of bytes read. Data is available in the member .buffer.
               Note if the returned number is equal to BLE_BUFSIZE, the internal
               buffer is full before reaching endline. User should continue to
               call this function a few more times.
*/
/******************************************************************************/
uint16_t Adafruit_BLE::readline(uint16_t timeout, boolean multiline)
{
  uint16_t replyidx = 0;

  while (timeout--) {
    while(available()) {
      char c =  read();
      //SerialDebug.println(c);
      if (c == '\r') continue;

      if (c == '\n') {
        if (replyidx == 0)   // the first '\n' is ignored
          continue;
        
        if (!multiline) {
          timeout = 0;         // the second 0x0A is the end of the line
          break;
        }
      }
      buffer[replyidx] = c;
      replyidx++;

      // Buffer is full
      if (replyidx >= BLE_BUFSIZE) {
        //if (_verbose) { SerialDebug.println("*overflow*"); }  // for my debuggin' only!
        timeout = 0;
        break;
      }
    }
    
    if (timeout == 0) break;
    delay(1);
  }
  buffer[replyidx] = 0;  // null term

  // Print out if is verbose
  if (_verbose && replyidx > 0)
  {
    SerialDebug.print(buffer);
    if (replyidx < BLE_BUFSIZE) SerialDebug.println();
  }

  return replyidx;
}

/******************************************************************************/
/*!
    @brief  Get (multiple) lines of response data into internal buffer.

    @param[in] period_ms
               period in milliseconds between each event scanning
    @return    None
*/
/******************************************************************************/
void Adafruit_BLE::loop(uint32_t period_ms)
{
  static TimeoutTimer tt;

  if ( tt.expired() )
  {
    tt.set(period_ms);

    bool v = _verbose;
    _verbose = false;

    uint8_t current_mode = _mode;

    // switch mode if necessary to execute command
    if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_COMMAND);

    println( F("AT+EVENTSTATUS") );
    readline();

    // parse event status system_event, gatts_event
    uint32_t system_event, gatts_event;
    char * p_comma = NULL;

    system_event = strtoul(this->buffer, &p_comma, 16);
    gatts_event  = strtoul(p_comma+1, NULL, 16);

    if ( this->_connect_callback && bitRead(system_event, EVENT_SYSTEM_CONNECT) )
    {
      this->_connect_callback();
    }

    if ( this->_disconnect_callback && bitRead(system_event, EVENT_SYSTEM_DISCONNECT) )
    {
      this->_disconnect_callback();
    }

    if ( this->_ble_uart_rx_callback && bitRead(system_event, EVENT_SYSTEM_BLE_UART_RX) )
    {
      //     uint8_t _verbose = true;
      println( F("AT+BLEUARTRX") );
      uint16_t len = readline();

      this->_ble_uart_rx_callback(this->buffer, len);

      waitForOK();
    }

    if ( this->_ble_midi_rx_callback && bitRead(system_event, EVENT_SYSTEM_BLE_MIDI_RX) )
    {

      //    uint8_t _verbose = true;
      println( F("AT+BLEMIDIRX") );
      uint16_t len = readline();

      this->_ble_midi_rx_callback( (uint8_t*) this->buffer, len);

      waitForOK();
    }

    // switch back if necessary
    if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_DATA);

    _verbose = v;
  }
}

/******************************************************************************/
/*!
    @brief  Set handle for connect callback

    @param[in] fp function pointer, NULL will discard callback
*/
/******************************************************************************/
void Adafruit_BLE::setHandleConnect( void (*fp) (void) )
{
  this->_connect_callback = fp;
  install_callback(fp != NULL, EVENT_SYSTEM_CONNECT, 0);
}

/******************************************************************************/
/*!
    @brief  Set handle for disconnection callback

    @param[in] fp function pointer, NULL will discard callback
*/
/******************************************************************************/
void Adafruit_BLE::setHandleDisconnect( void (*fp) (void) )
{
  this->_disconnect_callback = fp;
  install_callback(fp != NULL, EVENT_SYSTEM_DISCONNECT, 0);
}

/******************************************************************************/
/*!
    @brief  Set handle for BLE Uart Rx callback

    @param[in] fp function pointer, NULL will discard callback
*/
/******************************************************************************/
void Adafruit_BLE::setHandleBleUartRx( void (*fp) (char data[], uint16_t len) )
{
  this->_ble_uart_rx_callback = fp;
  install_callback(fp != NULL, EVENT_SYSTEM_BLE_UART_RX, 0);
}

/******************************************************************************/
/*!
    @brief  Set handle for BLE MIDI Rx callback

    @param[in] fp function pointer, NULL will discard callback
*/
/******************************************************************************/
void Adafruit_BLE::setHandleBleMidiRx( void (*fp) (uint8_t data[], uint16_t len) )
{
  this->_ble_midi_rx_callback = fp;
  install_callback(fp != NULL, EVENT_SYSTEM_BLE_MIDI_RX, 0);
}


