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

static inline char digit2ascii(uint8_t digit)
{
  return ( digit + ((digit) < 10 ? '0' : 'A') );
}

enum {
  EVENT_SYSTEM_CONNECT     = 0,
  EVENT_SYSTEM_DISCONNECT  = 1,

  EVENT_SYSTEM_BLE_UART_RX = 8,
  // 9 reserved

  EVENT_SYSTEM_BLE_MIDI_RX = 10,
  //  11 reserved
};

void Adafruit_BLE::install_callback(bool enable, int8_t system_id, int8_t gatts_id)
{
  bool v = _verbose;
  _verbose = true;

  uint8_t current_mode = _mode;

  // switch mode if necessary to execute command
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_COMMAND);

  print( enable ?  F("AT+EVENTENABLE=0x") : F("AT+EVENTDISABLE=0x") );
  print( (system_id < 0) ? 0 : bit(system_id), HEX );

  if ( gatts_id >= 0 )
  {
    print( F(",0x") );
    println( bit(gatts_id), HEX );
  }

  println();


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
  _ble_gatt_rx_callback = NULL;
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
    @brief  Disconnect if currently connected
*/
/******************************************************************************/
void Adafruit_BLE::disconnect(void)
{
  sendCommandCheckOK( F("AT+GAPDISCONNECT") );
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
bool Adafruit_BLE::isVersionAtLeast(const char * versionString)
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
    @brief  Convert buffer data to Byte Array String format such as 11-22-33-44
    e.g 0x11223344 --> 11-22-33-44

    @return number of character converted including dash '-'
*/
/******************************************************************************/
uint8_t Adafruit_BLE::convert2ByteArrayString(char *str, const uint8_t* buffer, uint8_t count)
{
  for(uint8_t i=0; i<count; i++)
  {
    uint8_t byte = *buffer++;
    *str++ = digit2ascii((byte & 0xF0) >> 4);
    *str++ = digit2ascii(byte & 0x0F);

    if (i != count-1) *str++ = '-';
  }

  return (count*3) - 1;
}

/******************************************************************************/
/*!
    @brief  Send a command from a SRAM string, and wait for OK or ERROR
*/
/******************************************************************************/
bool Adafruit_BLE::sendCommandCheckOK(const char cmd[], int32_t para_arr[], uint8_t para_count)
{
  bool result;
  uint8_t current_mode = _mode;

  // switch mode if necessary to execute command
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_COMMAND);

  // send command and integer parameters separated by comma
  print(cmd);
  for(uint8_t i=0; i<para_count; i++)
  {
    print(para_arr[i]);
    if (i != para_count - 1) print(',');
  }
  println(); // execute command

  result = waitForOK();

  // switch back if necessary
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_DATA);

  return result;
}

/******************************************************************************/
/*!
    @brief  Send a command from a SRAM string, and wait for OK or ERROR
*/
/******************************************************************************/
bool Adafruit_BLE::sendCommandCheckOK(const __FlashStringHelper *cmd, int32_t para_arr[], uint8_t para_count)
{
  bool result;
  uint8_t current_mode = _mode;

  // switch mode if necessary to execute command
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_COMMAND);

  // send command and integer parameters separated by comma
  print(cmd);
  for(uint8_t i=0; i<para_count; i++)
  {
    print(para_arr[i]);
    if (i != para_count - 1) print(',');
  }
  println(); // execute command

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

  do
  {
    rd = readline();

    uint16_t n = min(len, rd);
    memcpy(buf, buffer, n);

    buf += n;
    len -= n;
  } while ( (len > 0) && (rd == BLE_BUFSIZE) );

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
    @brief  Get raw binary data to internal buffer, only stop when encountering
            either "OK\r\n" or "ERROR\r\n" or timed out. Buffer does not contain
            OK or ERROR

    @param[in] timeout
               Timeout for each read() operation

    @return    The number of bytes read excluding OK, ERROR ending.
*/
/******************************************************************************/
uint16_t Adafruit_BLE::readraw(uint16_t timeout)
{
  uint16_t replyidx = 0;

  while (timeout--) {
    while(available()) {
      char c =  read();

      if (c == '\n')
      {
        // done if ends with "OK\r\n"
        if ( (replyidx >= 3) && !strncmp(this->buffer + replyidx-3, "OK\r", 3) )
        {
          replyidx -= 3; // chop OK\r
          timeout = 0;
          break;
        }
        // done if ends with "ERROR\r\n"
        else if ((replyidx >= 6) && !strncmp(this->buffer + replyidx-6, "ERROR\r", 6))
        {
          replyidx -= 6; // chop ERROR\r
          timeout = 0;
          break;
        }
      }

      this->buffer[replyidx] = c;
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
  this->buffer[replyidx] = 0;  // null term

  // Print out if is verbose
//  if (_verbose && replyidx > 0)
//  {
//    SerialDebug.print(buffer);
//    if (replyidx < BLE_BUFSIZE) SerialDebug.println();
//  }

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
void Adafruit_BLE::update(uint32_t period_ms)
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
    uint8_t tempbuf[BLE_BUFSIZE+1];
    uint32_t system_event, gatts_event;
    char * p_comma = NULL;

    system_event = strtoul(this->buffer, &p_comma, 16);
    gatts_event  = strtoul(p_comma+1, NULL, 16);

    //--------------------------------------------------------------------+
    // System Event
    //--------------------------------------------------------------------+
    if ( this->_connect_callback    && bitRead(system_event, EVENT_SYSTEM_CONNECT   ) ) this->_connect_callback();
    if ( this->_disconnect_callback && bitRead(system_event, EVENT_SYSTEM_DISCONNECT) ) this->_disconnect_callback();

    if ( this->_ble_uart_rx_callback && bitRead(system_event, EVENT_SYSTEM_BLE_UART_RX) )
    {
      // _verbose = true;
      println( F("AT+BLEUARTRX") );
      uint16_t len = readline(tempbuf, BLE_BUFSIZE);
      waitForOK();

      this->_ble_uart_rx_callback( (char*) tempbuf, len);
    }

    if ( this->_ble_midi_rx_callback && bitRead(system_event, EVENT_SYSTEM_BLE_MIDI_RX) )
    {
//      _verbose = true;
      while(1)
      {
        // use RAW command version
        println( F("AT+BLEMIDIRXRAW") );

        // readraw swallow OK/ERROR already
        uint16_t len = readraw();

        // break if there is no more MIDI event
        if ( len == 0 ) break;

        // copy to internal buffer for other usage !
        memcpy(tempbuf, this->buffer, len);

        // only support one full event for now
        if ( len == 5)
        {
          this->_ble_midi_rx_callback( *((uint16_t*) tempbuf), tempbuf[2], tempbuf[3], tempbuf[4]);
        }else
        {
          // TODO support multiple event (running events) parsing
        }
      }
    }

    //--------------------------------------------------------------------+
    // Gatt Event
    //--------------------------------------------------------------------+
    if ( this->_ble_gatt_rx_callback && gatts_event )
    {
//      _verbose = true;
      for(uint8_t charid=1; charid < 30; charid++)
      {
        if ( bitRead(gatts_event, charid-1) )
        {
          print( F("AT+GATTCHARRAW=") ); // use RAW command version
          println(charid);

          uint16_t len = readraw(); // readraw swallow OK/ERROR already
          memcpy(tempbuf, this->buffer, len);

          this->_ble_gatt_rx_callback(charid, tempbuf, len);
        }
      }
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
void Adafruit_BLE::setConnectCallback( void (*fp) (void) )
{
  this->_connect_callback = fp;
  install_callback(fp != NULL, EVENT_SYSTEM_CONNECT, -1);
}

/******************************************************************************/
/*!
    @brief  Set handle for disconnection callback

    @param[in] fp function pointer, NULL will discard callback
*/
/******************************************************************************/
void Adafruit_BLE::setDisconnectCallback( void (*fp) (void) )
{
  this->_disconnect_callback = fp;
  install_callback(fp != NULL, EVENT_SYSTEM_DISCONNECT, -1);
}

/******************************************************************************/
/*!
    @brief  Set handle for BLE Uart Rx callback

    @param[in] fp function pointer, NULL will discard callback
*/
/******************************************************************************/
void Adafruit_BLE::setBleUartRxCallback( void (*fp) (char data[], uint16_t len) )
{
  this->_ble_uart_rx_callback = fp;
  install_callback(fp != NULL, EVENT_SYSTEM_BLE_UART_RX, -1);
}

/******************************************************************************/
/*!
    @brief  Set handle for BLE MIDI Rx callback

    @param[in] fp function pointer, NULL will discard callback
*/
/******************************************************************************/
void Adafruit_BLE::setBleMidiRxCallback( bleMIDIRxCallback_t fp )
{
  this->_ble_midi_rx_callback = fp;
  install_callback(fp != NULL, EVENT_SYSTEM_BLE_MIDI_RX, -1);
}

/******************************************************************************/
/*!
    @brief  Set handle for BLE Gatt Rx callback

    @param[in] fp function pointer, NULL will discard callback
*/
/******************************************************************************/
void Adafruit_BLE::setBleGattRxCallback(int32_t chars_idx,  void (*fp) (int32_t, uint8_t[], uint16_t) )
{
  if ( chars_idx == 0) return;

  this->_ble_gatt_rx_callback = fp;
  install_callback(fp != NULL, -1, chars_idx-1);
}

