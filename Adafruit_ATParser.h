/**************************************************************************/
/*!
    @file     Adafruit_ATParser.h
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2016, Adafruit Industries (adafruit.com)
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

#ifndef _ADAFRUIT_ATPARSER_H_
#define _ADAFRUIT_ATPARSER_H_

#include <Arduino.h>
#include "utility/sdep.h"

// Class to facilitate sending AT Command and check response

#define BLUEFRUIT_MODE_COMMAND   HIGH
#define BLUEFRUIT_MODE_DATA      LOW
#define BLE_BUFSIZE              4*SDEP_MAX_PACKETSIZE


#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
#define SerialDebug SERIAL_PORT_USBVIRTUAL
#else
#define SerialDebug Serial
#endif


enum ATArgType
{
  AT_ARGTYPE_STRING,
  AT_ARGTYPE_BYTEARRAY,
  AT_ARGTYPE_INT32,
//  AT_ARGTYPE_UINT32,
//  AT_ARGTYPE_UINT16,
//  AT_ARGTYPE_UINT8,
//  AT_ARGTYPE_INT8,
};

class Adafruit_ATParser : public Stream
{
protected:
  uint8_t _mode;
  bool     _verbose;

  // internal function
  bool send_arg_get_resp(int32_t* reply, uint8_t argcount, ATArgType argtype[], const void* args[]);

public:
  Adafruit_ATParser(void);

  char buffer[BLE_BUFSIZE+1];

  uint8_t      getMode(void) { return _mode; }
  virtual bool setMode(uint8_t mode) = 0;

  // Auto print out TX & RX data to normal Serial
  void verbose(bool enable) { _verbose = enable; }

  bool atcommand_full(const char cmd[]               , int32_t* reply, uint8_t argcount, ATArgType argtype[], const void* args[]);
  bool atcommand_full(const __FlashStringHelper *cmd , int32_t* reply, uint8_t argcount, ATArgType argtype[], const void* args[]);

  //--------------------------------------------------------------------+
  // Without Reply
  //--------------------------------------------------------------------+
  bool atcommand(const char cmd[]               ) { return this->atcommand_full(cmd, NULL, 0, NULL, NULL); }
  bool atcommand(const __FlashStringHelper *cmd ) { return this->atcommand_full(cmd, NULL, 0, NULL, NULL); }

  //------------- One integer argument -------------//
  bool atcommand(const char cmd[]               , int32_t para1)
  {
    ATArgType type[] = { AT_ARGTYPE_INT32 };
    const void* args[] = { (void*) para1 };
    return this->atcommand_full(cmd, NULL, 1, type, args);
  }

  bool atcommand(const __FlashStringHelper *cmd , int32_t para1)
  {
    ATArgType type[] = { AT_ARGTYPE_INT32 };
    const void* args[] = { (void*) para1 };
    return this->atcommand_full(cmd, NULL, 1, type, args);
  }

  //------------- Two integer arguments -------------//
  bool atcommand(const char cmd[]               , int32_t para1, int32_t para2)
  {
    ATArgType type[] = { AT_ARGTYPE_INT32, AT_ARGTYPE_INT32 };
    const void* args[] = { (void*) para1, (void*) para2 };
    return this->atcommand_full(cmd, NULL, 1, type, args);
  }

  bool atcommand(const __FlashStringHelper *cmd , int32_t para1, int32_t para2)
  {
    ATArgType type[] = { AT_ARGTYPE_INT32, AT_ARGTYPE_INT32 };
    const void* args[] = { (void*) para1, (void*) para2 };
    return this->atcommand_full(cmd, NULL, 1, type, args);
  }


  //------------- With Reply -------------//
  //--------------------------------------------------------------------+
  // With Reply
  //--------------------------------------------------------------------+
  bool atcommandIntReply(const char cmd[], int32_t* reply)               { return this->atcommand_full(cmd, reply, 0, NULL, NULL); }
  bool atcommandIntReply(const __FlashStringHelper *cmd, int32_t* reply) { return this->atcommand_full(cmd, reply, 0, NULL, NULL); }

  bool waitForOK(void);

  //--------------------------------------------------------------------+
  //
  //--------------------------------------------------------------------+
  // Read one line of response into internal buffer
  uint16_t readline(uint16_t timeout, boolean multiline = false);
  uint16_t readline(void)
  {
    return readline(_timeout, false);
  }

  // Read one line of response into provided buffer
  uint16_t readline(char    * buf, uint16_t bufsize);
  uint16_t readline(uint8_t * buf, uint16_t bufsize)
  {
    return readline( (char*) buf, bufsize);
  }

  // read one line and convert the string to integer number
  int32_t readline_parseInt(void);

  uint16_t readraw(uint16_t timeout);
  uint16_t readraw(void)
  {
    return readraw(_timeout);
  }

};

#endif /* _ADAFRUIT_ATPARSER_H_ */
