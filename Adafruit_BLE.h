/**************************************************************************/
/*!
    @file     Adafruit_BLE.h
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

#ifndef _Adafruit_BLE_H_
#define _Adafruit_BLE_H_

#include <stdint.h>
#include <Arduino.h>
#include "utility/sdep.h"
#include "utility/errors.h"
#include "utility/TimeoutTimer.h"

#define BLE_DEFAULT_TIMEOUT      250
#define BLE_BUFSIZE              4*SDEP_MAX_PACKETSIZE

#define ASSERT(condition, err)    if ( !(condition) ) return err;

class Adafruit_BLE : public Stream
{
  protected:
    bool     _verbose;
    uint16_t _timeout;

  public:
    char buffer[BLE_BUFSIZE+1];


    // Auto print out TX & RX data to normal Serial
    void verbose(bool enable) { _verbose = enable; }

    // Functions implemented in this base class
    bool reset(void);
    bool factoryReset(void);
    void info(void);
    bool echo(bool enable);
    bool waitForOK(void);

    bool isConnected(void);

    bool sendCommandCheckOK(const __FlashStringHelper *cmd);
    bool sendCommandWithIntReply(const __FlashStringHelper *cmd, uint32_t *reply);

    // read one line from stream into buffer
    size_t readln( char *buffer, size_t length);
    size_t readln( uint8_t *buffer, size_t length)
    {
      return readln( (char *) buffer, length );
    }

    void readln(void);

    uint16_t readline(uint16_t timeout, boolean multiline = false);

    int32_t readln_parseInt(void);
};

//struct GattServer_t
//{
//  bool clear(void)
//  {
//    ASSERT( sendATCommand("AT+GATTCLEAR"), false);
//    ASSERT( getATResponse(), false);
//
//    return true;
//  }
//};
#endif /* _Adafruit_BLE_H_ */
