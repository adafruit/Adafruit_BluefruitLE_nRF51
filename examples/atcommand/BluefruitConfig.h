/*==========================================================================
                        If you are using Software Serial
    -----------------------------------------------------------------------
    The following macros declare the pins used for SW serial, you should
    use these pins if you are connecting the UART Friend to an UNO
    -----------------------------------------------------------------------*/
#define BLUEFRUIT_SWUART_RXD_PIN        9    // Required for software serial!
#define BLUEFRUIT_SWUART_TXD_PIN        10   // Required for software serial!
#define BLUEFRUIT_UART_CTS_PIN          11   // Required for software serial!
#define BLUEFRUIT_UART_RTS_PIN          -1   // Optional, set to -1 if unused

/*==========================================================================
                        If you are using Hardware Serial
    -----------------------------------------------------------------------
    The following macros declare the Serial port you are using. Uncomment this
    line if you are connecting the BLE to Leonardo/Micro or Flora
    -----------------------------------------------------------------------*/
#ifdef Serial1    // this makes it not complain on compilation if there's no Serial1
  #define BLUEFRUIT_HWSERIAL_NAME           Serial1
#endif

/*== Other recommended pins for Serial/UART mode selection
    The following sets the optional Mode pin, its recommended but not required
    -----------------------------------------------------------------------*/
#define BLUEFRUIT_UART_MODE_PIN         12   // Optional but recommended, set to -1 if unused


/*==========================================================================
                        If you are using Hardware SPI
    -----------------------------------------------------------------------*/
#define BLUEFRUIT_SPI_CS             10
#define BLUEFRUIT_SPI_IRQ            9
#define BLUEFRUIT_SPI_RST            8       // Optional but recommended, set to -1 if unused


/*====================== SKETCH SETTINGS
    READ_BUFSIZE            Size of the read buffer for incoming data
    VERBOSE_MODE            If set to true enables debugging output
    -----------------------------------------------------------------------*/
    #define BUFSIZE                         128
    #define VERBOSE_MODE                    true
/*=========================================================================*/
