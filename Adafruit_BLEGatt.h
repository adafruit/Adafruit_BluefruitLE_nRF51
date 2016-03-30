/**************************************************************************/
/*!
    @file     Adafruit_BLEGatt.h
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

#ifndef _ADAFRUIT_BLEGATT_H_
#define _ADAFRUIT_BLEGATT_H_

#include <Arduino.h>
#include "Adafruit_BLE.h"

enum GattCharsDataType_t
{
  GATT_CHARS_DATATYP_AUTO = 0,
  GATT_CHARS_DATATYPE_INTEGER,
  GATT_CHARS_DATATYPE_STRING,
  GATT_CHARS_DATATYPE_BYTEARRAY,
};

#define GATT_CHARS_PROPERTIES_BROADCAST       bit(0)
#define GATT_CHARS_PROPERTIES_READ            bit(1)
#define GATT_CHARS_PROPERTIES_WRITE_WO_RESP   bit(2)
#define GATT_CHARS_PROPERTIES_WRITE           bit(3)
#define GATT_CHARS_PROPERTIES_NOTIFY          bit(4)
#define GATT_CHARS_PROPERTIES_INDICATE        bit(5)

struct GattPresentationFormat
{
  uint8_t  format;
  int8_t   exponent;
  uint16_t unit;
  uint8_t  name_space;
  uint16_t desc;
};

class Adafruit_BLEGatt
{
private:
  Adafruit_BLE& _ble;

  uint8_t addChar_internal(uint8_t uuid[], uint8_t uuid_len, uint8_t properties, uint8_t min_len, uint8_t max_len, GattCharsDataType_t datatype, const char* description, const GattPresentationFormat* presentFormat);

public:
  char* buffer; // alias to ble's buffer

  Adafruit_BLEGatt(Adafruit_BLE& ble);

  bool    clear(void);

  uint8_t addService(uint16_t uuid16);
  uint8_t addService(uint8_t uuid128[]);

  uint8_t addCharacteristic(uint16_t uuid16  , uint8_t properties, uint8_t min_len, uint8_t max_len, GattCharsDataType_t datatype, const char* description = NULL, const GattPresentationFormat* presentFormat = NULL);
  uint8_t addCharacteristic(uint8_t uuid128[], uint8_t properties, uint8_t min_len, uint8_t max_len, GattCharsDataType_t datatype, const char* description = NULL, const GattPresentationFormat* presentFormat = NULL);

  //------------- Get Characteristic -------------//
  uint8_t  getChar(uint8_t charID);
  uint8_t  getChar(uint8_t charID, uint8_t* buf, uint8_t bufsize);

  uint8_t  getCharInt8(uint8_t charID)
  {
    if ( this->getChar(charID) < sizeof(uint8_t) ) return 0;
    uint8_t result;
    memcpy(&result, this->buffer, sizeof(result));
    return result;
  }

  uint16_t getCharInt16(uint8_t charID)
  {
    if ( this->getChar(charID) < sizeof(uint16_t) ) return 0;
    uint16_t result;
    memcpy(&result, this->buffer, sizeof(result));
    return result;
  }

  uint32_t getCharInt32(uint8_t charID)
  {
    if ( this->getChar(charID) < sizeof(uint32_t) ) return 0;
    uint32_t result;
    memcpy(&result, this->buffer, sizeof(result));
    return result;
  }

  char*    getCharInStr(uint8_t charID)
  {
    if ( this->getChar(charID) == 0 ) return NULL;
    return this->buffer;
  }

  //------------- Set Characteristic -------------//
  bool    setChar(uint8_t charID, uint8_t const data[], uint8_t size);
  bool    setChar(uint8_t charID, char const *  str);

  bool    setChar(uint8_t charID, uint8_t  data8 ) { return this->setChar(charID, (uint8_t*) &data8, 1); }
  bool    setChar(uint8_t charID, int8_t   data8 ) { return this->setChar(charID, (uint8_t*) &data8, 1); }

  bool    setChar(uint8_t charID, uint16_t data16) { return this->setChar(charID, (uint8_t*) &data16, 2); }
  bool    setChar(uint8_t charID, int16_t  data16) { return this->setChar(charID, (uint8_t*) &data16, 2); }

  bool    setChar(uint8_t charID, uint32_t data32) { return this->setChar(charID, (uint8_t*) &data32, 4); }
  bool    setChar(uint8_t charID, int32_t  data32) { return this->setChar(charID, (uint8_t*) &data32, 4); }
};

#endif /* _ADAFRUIT_BLEGATT_H_ */
