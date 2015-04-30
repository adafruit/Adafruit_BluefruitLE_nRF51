/**************************************************************************/
/*!
    @file     Adafruit_BLE_HWSPI.cpp
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
#include "Adafruit_BLE_HWSPI.h"
#include <Arduino.h>
#include <stdlib.h>

/******************************************************************************/
/*!
    @brief  ToDo: Interrupt handler for the IRQ pin
*/
/******************************************************************************/
static void bleModule_IRQHandler(void)
{
  // TODO interrupt for IRQ
}

/******************************************************************************/
/*!
    @brief Instantiates a new instance of the Adafruit_BLE_HWSPI class

    @param[in]  csPin
                The location of the CS pin for the SPI interface
    @param[in]  irqPin
                The location of the HW IRQ pin (pin 2 or pin 3 on the Arduino
                Uno). This must be a HW interrupt pin!
    @param[in]  rstPin
*/
/******************************************************************************/
Adafruit_BLE_HWSPI::Adafruit_BLE_HWSPI(int8_t csPin, int8_t irqPin, int8_t rstPin) :
    m_rx_fifo(m_rx_buffer, sizeof(m_rx_buffer), 1, true)
{
  memclr(&m_responseHeader, sizeof(sdepMsgHeader_t));

  m_cs_pin  = csPin;
  m_irq_pin = irqPin;
  m_rst_pin = rstPin;

  m_tx_count = 0;
  _verbose   = false;
  _timeout   = BLE_DEFAULT_TIMEOUT;
}


/******************************************************************************/
/*!
    @brief Initialize the HW to enable communication with the BLE module

    @return Returns 'true' if everything initialised correctly, otherwise
            'false' if there was a problem during HW initialisation. If
            'irqPin' is not a HW interrupt pin false will be returned.
*/
/******************************************************************************/
bool Adafruit_BLE_HWSPI::begin(void)
{
  // Determine the HW irq pin number
  uint8_t irq_num;
  irq_num = 0xff;
  for (uint8_t i=0; i<sizeof(dreqinttable); i+=2)
  {
    if (m_irq_pin == dreqinttable[i])
    {
      irq_num = dreqinttable[i+1];
      break;
    }
  }

  if (irq_num == 0xff) return false;

  pinMode(m_irq_pin, INPUT);
  attachInterrupt(irq_num, bleModule_IRQHandler, /*CHANGE*/ RISING);

  // Set CS pin to output and de-assert by default
  digitalWrite(m_cs_pin, HIGH);
  pinMode(m_cs_pin, OUTPUT);

  // Initialise SPI (Mode 0), MSB first
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  // nrf51 may not keep up with the SPI speed
  SPI.setClockDivider(SPI_CLOCK_SPEED);

  // Send Initialize command (this will cause Bluefruit to reset)
  if ( !sendInitializePattern() ) return false;
  delay(1000);

  return true;
}

/******************************************************************************/
/*!
    @brief  Uninitializes the SPI interface
*/
/******************************************************************************/
void Adafruit_BLE_HWSPI::end(void)
{
  SPI.end();
//  detachInterrupt()
}

/******************************************************************************/
/*!
    @brief  Performs a system reset using RST pin
*/
/******************************************************************************/
bool Adafruit_BLE_HWSPI::hwreset(void)
{
  if (m_rst_pin < 0) return false;

  // pull the RST to GND for 20 ms
  pinMode(m_rst_pin, OUTPUT);
  digitalWrite(m_rst_pin, HIGH);
  digitalWrite(m_rst_pin, LOW);
  delay(20);
  digitalWrite(m_rst_pin, HIGH);

  // delay 1 second for Bluefruit
  delay(1000);

  return true;
}

/******************************************************************************/
/*!
    @brief  Read data from Ble module's SPI interface.  
            This function will retry if the BLE module is not ready
            (response = SPI_IGNORED_BYTE), or terminate when 'length' bytes
            have been read, SPI_OVERREAD_BYTE is returned or 'timeout_ms' have
            passed.

    @param[out] buf
                buffer to store read data
    @param[in]  length 
                number of bytes to be read

    @return     The number of bytes successfully read
*/
/******************************************************************************/
uint32_t Adafruit_BLE_HWSPI::bus_read(uint8_t *buf, uint32_t length)
{
  TimeoutTimer tt(_timeout);

  SPI_CS_ENABLE();
  for(uint32_t count=0; count<length && !tt.expired(); count++)
  {
    uint8_t rd = SPI_OVERREAD_BYTE;

    while( !tt.expired() )
    {
      rd = SPI.transfer(0xff);

      if (rd != SPI_IGNORED_BYTE && rd != SPI_OVERREAD_BYTE) break;

      // blemodule is busy processing, wait a bit before retry
      SPI_CS_DISABLE();
      delayMicroseconds(SPI_DEFAULT_DELAY_US);
      SPI_CS_ENABLE();
    }

    if (rd == SPI_OVERREAD_BYTE) return count;
    *buf++ = rd;
  }
  SPI_CS_DISABLE();

  return length;
}

/******************************************************************************/
/*!
    @brief  Write data to BLE module's SPI interface.
            This function will retry if the BLE module is not ready
            (response = SPI_IGNORED_BYTE), or terminate when 'length' bytes
            have been written or 'timeout_ms' have passed.

    @param[out] buf
                Buffer containing the data to write
    @param[in]  length 
                Number of bytes to write

    @return     The number of bytes written
*/
/******************************************************************************/
uint32_t Adafruit_BLE_HWSPI::bus_write (uint8_t *buf, uint32_t length)
{
  uint32_t count=0;
  TimeoutTimer tt(_timeout);

  SPI_CS_ENABLE();
  while ( (count < length) && !tt.expired() )
  {
    // Keep re-sending if SPI_IGNORED_BYTE is received
    while( !tt.expired() )
    {
      uint8_t rd = SPI.transfer(*buf);

      if (rd != SPI_IGNORED_BYTE) break;

      // blemodule is busy processing, wait a bit before retry
      SPI_CS_DISABLE();
      delayMicroseconds(SPI_DEFAULT_DELAY_US);
      SPI_CS_ENABLE();
    }

    buf++;
    count++;
  }
  SPI_CS_DISABLE();

  return count;
}

/******************************************************************************/
/*!
    @brief Send initialize pattern to Bluefruit LE to force a reset. This pattern
    follow the SDEP command syntax with command_id = SDEP_CMDTYPE_INITIALIZE.
    The command has NO response, and is expected to complete within 1 second
*/
/******************************************************************************/
bool Adafruit_BLE_HWSPI::sendInitializePattern(void)
{
  sdepMsgHeader_t msgHeader;

  msgHeader.msg_type    = SDEP_MSGTYPE_COMMAND;
  msgHeader.cmd_id_high = highByte(SDEP_CMDTYPE_INITIALIZE);
  msgHeader.cmd_id_low  = lowByte(SDEP_CMDTYPE_INITIALIZE);
  msgHeader.length      = 0;
  msgHeader.more_data   = 0;

  return (sizeof(sdepMsgHeader_t) == bus_write((uint8_t*)&msgHeader, sizeof(sdepMsgHeader_t)));
}

/******************************************************************************/
/*!
    @brief  Send out an packet with data in m_tx_buffer

    @param[in]  more_data
                More Data bitfield, 0 indicates this is not end of transfer yet
*/
/******************************************************************************/
bool Adafruit_BLE_HWSPI::sendPacket(uint8_t more_data)
{
  // flush old response before sending the new command
  if (more_data == 0)
  {
    memclr(&m_responseHeader, sizeof(sdepMsgHeader_t));
    flush();
  }

  sdepMsgHeader_t msgHeader;

  msgHeader.msg_type    = SDEP_MSGTYPE_COMMAND;
  msgHeader.cmd_id_high = highByte(SDEP_CMDTYPE_AT_WRAPPER);
  msgHeader.cmd_id_low  = lowByte(SDEP_CMDTYPE_AT_WRAPPER);
  msgHeader.length      = m_tx_count;
  msgHeader.more_data   = (m_tx_count == SDEP_MAX_PACKETSIZE) ? more_data : 0;

  // Send the SDEP header
  ASSERT( sizeof(sdepMsgHeader_t) == bus_write((uint8_t*)&msgHeader, sizeof(sdepMsgHeader_t)), false );

  // Send the command payload
  ASSERT ( msgHeader.length == bus_write(m_tx_buffer, msgHeader.length), false );

  return true;
}

/******************************************************************************/
/*!
    @brief Print API, either buffered data internally or send SDEP packet to bus
    if possible. An \r, \n is command terminator will force the packet to be sent

    @param[in]  c
                Character to send
*/
/******************************************************************************/
size_t Adafruit_BLE_HWSPI::write(uint8_t c)
{
  // Final packet due to \r or \n terminator
  if (c == '\r' || c == '\n')
  {
    if (m_tx_count > 0)
    {
      sendPacket(0);
      m_tx_count = 0;
    }
  }
  // More than max packet bufferd --> send with more_data = 1
  else if (m_tx_count == SDEP_MAX_PACKETSIZE)
  {
    sendPacket(1);

    m_tx_buffer[0] = c;
    m_tx_count = 1;
  }
  // Not enough data, continue to buffer
  else
  {
    m_tx_buffer[m_tx_count++] = c;
  }

  if (_verbose) Serial.print((char) c);

  return 1;
}

/******************************************************************************/
/*!
    @brief Check if the response from the previous command is ready

    @return 'true' if a response is ready, otherwise 'false'
*/
/******************************************************************************/
int Adafruit_BLE_HWSPI::available(void)
{
  return digitalRead(m_irq_pin);
}

/******************************************************************************/
/*!
    @brief Get a byte from response data, perform SPI transaction if needed

    @return -1 if no data is available
*/
/******************************************************************************/
int Adafruit_BLE_HWSPI::read(void)
{
  // Read data from Bluefruit if possible
  if ( digitalRead(m_irq_pin) ) getResponse();

  uint8_t ch;
  return m_rx_fifo.read(&ch) ? ((int) ch) : EOF;
}

/******************************************************************************/
/*!
    @brief Get a byte from response without removing it, perform SPI transaction
           if needed

    @return -1 if no data is available
*/
/******************************************************************************/
int Adafruit_BLE_HWSPI::peek(void)
{
  // Read data from Bluefruit if possible
  if ( digitalRead(m_irq_pin) ) getResponse();

  uint8_t ch;
  return m_rx_fifo.peek(&ch) ? ch : EOF;
}

/******************************************************************************/
/*!
    @brief Flush current response data in the internal FIFO

    @return -1 if no data is available
*/
/******************************************************************************/
void Adafruit_BLE_HWSPI::flush(void)
{
  m_rx_fifo.clear();
}

/******************************************************************************/
/*!
    @brief  Try to perform an full AT response transfer from Bluefruit, or execute
            as many SPI transaction as internal FIFO can hold up.

    @note   If verbose is enabled, all the received data will be print to Serial

    @return
      - true  : if succeeded
      - false : if failed
*/
/******************************************************************************/
bool Adafruit_BLE_HWSPI::getResponse(void)
{
  // Blocking wait until IRQ is asserted
  while ( !digitalRead(m_irq_pin) ) {}

  // There is data from Bluefruit & enough room in the fifo
  while ( digitalRead(m_irq_pin) &&
          ( sizeof(m_rx_buffer) - m_rx_fifo.count() ) >= SDEP_MAX_PACKETSIZE
  )
  {
    // Get a SDEP packet
    uint8_t buffer[SDEP_MAX_PACKETSIZE+1];
    int len = getPacket(buffer);
    if ( len < 0 ) return false;

    // Write to fifo
    m_rx_fifo.write_n(buffer, len);

    if (_verbose)
    {
      buffer[len] = 0;
      Serial.print( (char*) buffer);

      // small delay to make sure character is printed out
      delay(1);
    }
  }

  return true;
}

/******************************************************************************/
/*!
    @brief      Perform a single SPI SDEP transaction and is used by getReponse to
                get a full response composed of multiple packets.

    @param[in]  buffer
                Memory location where payload is copied to

    @return number of bytes in SDEP payload
*/
/******************************************************************************/
int Adafruit_BLE_HWSPI::getPacket(uint8_t* buffer)
{
  // Wait for SDEP_MSGTYPE_RESPONSE
  uint8_t sync=0;
  do {
    if ( 1 != bus_read(&sync, 1) ) return -1;
  } while(sync != SDEP_MSGTYPE_RESPONSE && sync != SDEP_MSGTYPE_ERROR);

  m_responseHeader.msg_type = sync;

  // Error Message Response only has 3 bytes (no payload's length)
  if (m_responseHeader.msg_type == SDEP_MSGTYPE_ERROR)
  {
    bus_read((uint8_t*)&m_responseHeader.cmd_id, 2);

    uint32_t error_code = (uint32_t) word(m_responseHeader.cmd_id_high, m_responseHeader.cmd_id_low);
    (void) error_code;

    return -1;
  }

  // Normal Response Message, first get header
  if ( 3 != bus_read((uint8_t*)&m_responseHeader.cmd_id, 3) ) return -1;
  if ( m_responseHeader.cmd_id != SDEP_CMDTYPE_AT_WRAPPER) return -1;

  // Get payload
  int len = bus_read( buffer, m_responseHeader.length);
  if ( len != m_responseHeader.length ) return -1;

  return len;
}

