/**************************************************************************/
/*!
    @file     Adafruit_BLE_HWSPI.h
    @author   hathach, ktown (Adafruit Industries)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2015, Adafruit Industries (adafruit.com)
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
#ifndef _ADAFRUIT_BLE_HWSPI_H_
#define _ADAFRUIT_BLE_HWSPI_H_

#include <Adafruit_BLE.h>
#include <SPI.h>
#include "utility/Adafruit_FIFO.h"

#define SPI_CLOCK_SPEED           SPI_CLOCK_DIV4
#define SPI_CS_ENABLE()           do { digitalWrite(m_cs_pin, LOW); delayMicroseconds(5); }while(0)
#define SPI_CS_DISABLE()          digitalWrite(m_cs_pin, HIGH)

#define SPI_IGNORED_BYTE          0xFEu /**< SPI default character. Character clocked out in case of an ignored transaction. */
#define SPI_OVERREAD_BYTE         0xFFu /**< SPI over-read character. Character clocked out after an over-read of the transmit buffer. */
#define SPI_DEFAULT_DELAY_US      50

#define memclr(buffer, size)  memset(buffer, 0, size)

static const uint8_t dreqinttable[] =
{
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined(__AVR_ATmega8__)
  2, 0,
  3, 1,
#elif defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  2, 0,
  3, 1,
  21, 2,
  20, 3,
  19, 4,
  18, 5,
#elif  defined(__AVR_ATmega32U4__) && defined(CORE_TEENSY)
  5, 0,
  6, 1,
  7, 2,
  8, 3,
#elif  defined(__AVR_AT90USB1286__) && defined(CORE_TEENSY)
  0, 0,
  1, 1,
  2, 2,
  3, 3,
  36, 4,
  37, 5,
  18, 6,
  19, 7,
#elif  defined(__arm__) && defined(CORE_TEENSY)
  0, 0, 1, 1, 2, 2, 3, 3, 4, 4,
  5, 5, 6, 6, 7, 7, 8, 8, 9, 9,
  10, 10, 11, 11, 12, 12, 13, 13, 14, 14,
  15, 15, 16, 16, 17, 17, 18, 18, 19, 19,
  20, 20, 21, 21, 22, 22, 23, 23, 24, 24,
  25, 25, 26, 26, 27, 27, 28, 28, 29, 29,
  30, 30, 31, 31, 32, 32, 33, 33,
#elif  defined(__AVR_ATmega32U4__)
  7, 4,
  3, 0,
  2, 1,
  0, 2,
  1, 3,
#elif defined(__arm__) && defined(__SAM3X8E__) // Arduino Due
  0, 0, 1, 1, 2, 2, 3, 3, 4, 4,
  5, 5, 6, 6, 7, 7, 8, 8, 9, 9,
  10, 10, 11, 11, 12, 12, 13, 13, 14, 14,
  15, 15, 16, 16, 17, 17, 18, 18, 19, 19,
  20, 20, 21, 21, 22, 22, 23, 23, 24, 24,
  25, 25, 26, 26, 27, 27, 28, 28, 29, 29,
  30, 30, 31, 31, 32, 32, 33, 33, 34, 34,
  35, 35, 36, 36, 37, 37, 38, 38, 39, 39,
  40, 40, 41, 41, 42, 42, 43, 43, 44, 44,
  45, 45, 46, 46, 47, 47, 48, 48, 49, 49,
  50, 50, 51, 51, 52, 52, 53, 53, 54, 54,
  55, 55, 56, 56, 57, 57, 58, 58, 59, 59,
  60, 60, 61, 61, 62, 62, 63, 63, 64, 64,
  65, 65, 66, 66, 67, 67, 68, 68, 69, 69,
  70, 70, 71, 71,
#endif
};

class Adafruit_BLE_HWSPI : public Adafruit_BLE
{
  private:
    // Hardware Pin
    int8_t          m_cs_pin;
    int8_t          m_irq_pin;
    int8_t          m_rst_pin;

    // TX
    uint8_t         m_tx_buffer[SDEP_MAX_PACKETSIZE];
    uint8_t         m_tx_count;

    // RX
    sdepMsgHeader_t m_responseHeader;
    uint8_t         m_rx_buffer[BLE_BUFSIZE];
    Adafruit_FIFO   m_rx_fifo;

    // Low level transportation I/O functions
    uint32_t bus_read(uint8_t *buf, uint32_t length);
    uint32_t bus_write(uint8_t *buf, uint32_t length);

    bool    sendInitializePattern(void);
    bool    sendPacket(uint8_t more_data);
    int     getPacket(uint8_t* buffer);

    bool    getResponse(void);

  public:
    // Constructor
    Adafruit_BLE_HWSPI(int8_t csPin, int8_t irqPin, int8_t rstPin = -1);

    // HW initialisation
    bool hwreset(void);
    bool begin();
    void end(void);

    // Class Print virtual function Interface
    virtual size_t write(uint8_t c);

    // pull in write(str) and write(buf, size) from Print
    using Print::write;

    // Class Stream interface
    virtual int  available(void);
    virtual int  read(void);
    virtual void flush(void);
    virtual int  peek(void);
};

#endif
