This is a simple patch for add support to Debrick Utility a new device Bus Pirate.
Parallel port are disappearing so bus pirate is an alternative. Is very slow, I need to improve it.


    BP   - JTAG
    GND  - GND
    MOSI - TDI  => output BP
    MISO - TDO  <= input BP
    CLK  - TCK  => output BP
    CS   - TMS  => output BP
    N/A  - TRST
    N/A  - RTCK
    AUX  - SRST => // output BP not used

