# NDOF BNO055 Example

This example is intended to be used with a board like the [Bluefruit LE Micro](https://www.adafruit.com/product/2661).

It will read data samples from the [BNO055](https://www.adafruit.com/products/2472)
at a fixed rate, and transmit the data over the air using the BLE UART service.

# Requirements

To use this example, you will also need the following libraries installed on
your system:

- [Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor)
- [Adafruit_BNO055](https://github.com/adafruit/Adafruit_BNO055)

## Wiring

Connect your Bluefruit LE Micro to the BNO055 breakout using I2C as follows:

- VIN on the BNO055 to USB on the Bluefruit LE Micro
- GND on the BNO055 to GND on the Bluefruit LE Micro
- SDA on the BNO055 to SDA on the Bluefruit LE Micro
- SCL on the BNO055 to SCL on the Bluefruit LE Micro
