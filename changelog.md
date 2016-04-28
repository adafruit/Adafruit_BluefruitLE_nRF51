# Changelog

## 1.9

### Features

- Add Adafruit_ATParser class helper to facilitate sending & receiving at commands
	- .atcommand() : without reply (several variants for input parameters)
	- .atcommandIntReply() with integer reply (several variants for input parameters)
	- .atcommand_full() : for general command execution
	- .printByteArray() : to print Byte Array format AA-BB-CC from a buffer. Useful for executing AT command.
	- .waitForOK() : use separate temporary buffer to avoid overwrite the response contents.
- Callback support to Adafruit_BLE class
	- Supported Events are Connect, Disconnect, Uart RX, MIDI Rx, GattChar Rx, those are set uing .setBleUartRxCallback(), .setBleMidiRxCallback(), .setBleGattRxCallback()
	- .update(ms) : must be placed in loop(), ms is the interval in milliseconds to poll for new event
	- Check the examples/callbacks sketch for more details
- Add Adafruit_BLEGatt class to facilitate GATT server adding service, characteristics as well as read/write, callback for characteristics
	- Add User Description & Presentation format support for characteristics.
	- Add BLEDataType_t for characteristics
	- Check the example/healththermometer for how to use Adafruit_BLEGatt class
- Add BLE MIDI service support with Adafruit_BLEMIDI  class
	-  Check the examples/midi for more details  
- Add BLE Battery service support with Adafruit_BLEBattery class
	- Check the example/battery for more details
- Add BLE Eddystone class to facilitate using Eddystone beacon
	- Check the example/eddystone for more details
- Add User NVM data support up to 256 bytes with .writeNVM() & .readNVM() to Adafruit_BLE class. User can use this small portion to store application specific data.
	- Check the example/nvmdata for more details
- Adafruit_BLE class
	- add setAdvData() helper to advertise custom data