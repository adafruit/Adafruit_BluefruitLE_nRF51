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

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL) 
#define SerialDebug SERIAL_PORT_USBVIRTUAL
#else
#define SerialDebug Serial
#endif

#define BLUEFRUIT_MODE_COMMAND   HIGH
#define BLUEFRUIT_MODE_DATA      LOW
#define BLE_DEFAULT_TIMEOUT      250
#define BLE_BUFSIZE              4*SDEP_MAX_PACKETSIZE

#define ASSERT(condition, err)    if ( !(condition) ) return err;

class Adafruit_BLE : public Stream
{
  protected:
    enum
    {
      BLUEFRUIT_TRANSPORT_INVALID,
      BLUEFRUIT_TRANSPORT_HWUART,
      BLUEFRUIT_TRANSPORT_SWUART,
      BLUEFRUIT_TRANSPORT_HWSPI,
      BLUEFRUIT_TRANSPORT_SWSPI,
    };

    bool     _verbose;
    uint8_t  _mode;
    uint16_t _timeout;
    uint8_t  _physical_transport;

  public:
    // Constructor
    Adafruit_BLE(void);
    char buffer[BLE_BUFSIZE+1];

    // Auto print out TX & RX data to normal Serial
    void verbose(bool enable) { _verbose = enable; }

    // Physical transportation checking
    bool isTransportHwUart (void) { return _physical_transport == BLUEFRUIT_TRANSPORT_HWUART; }
    bool isTransportSwUart (void) { return _physical_transport == BLUEFRUIT_TRANSPORT_SWUART; }
    bool isTransportUart   (void) { return isTransportHwUart() || isTransportSwUart();        }

    bool isTransportHwSpi  (void) { return _physical_transport == BLUEFRUIT_TRANSPORT_HWSPI;  }
    bool isTransportSwSpi  (void) { return _physical_transport == BLUEFRUIT_TRANSPORT_SWSPI;  }
    bool isTransportSpi    (void) { return isTransportHwSpi() || isTransportSwSpi();          }

    // Functions implemented in this base class
    bool reset(void);
    bool factoryReset(void);
    void info(void);
    bool echo(bool enable);
    bool waitForOK(void);
    bool isConnected(void);
    bool isVersionAtLeast(char * versionString);
    void disconnect(void);

    virtual bool setMode(uint8_t mode) = 0;

    bool sendCommandCheckOK(const __FlashStringHelper *cmd);
    bool sendCommandCheckOK(const char cmd[]);

    bool sendCommandWithIntReply(const __FlashStringHelper *cmd, int32_t *reply);
    bool sendCommandWithIntReply(const char cmd[], int32_t *reply);

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
