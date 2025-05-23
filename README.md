
## ESP32-WROOM-32 Pin Connections

ESP32 -> INMP441 (Microphone)
-----------------------------
G32 (Pin 32) -> BCLK
G25 (Pin 25) -> WS (shared with DAC)
G33 (Pin 33) <- SD
GND          -> L/R (to select Left channel)
GND          -> GND
3.3V         -> VDD

ESP32 -> MAX98357 (DAC)
-----------------------
G26 (Pin 26) -> BCLK
G27 (Pin 27) -> WS (shared with Mic)
G14 (Pin 14) -> DIN
G21 (Pin 21) -> SD (optional - connect to control shutdown)
                Leave SD pin disconnected or tied to VDD for always-on
GND          -> GND
3.3V         -> VDD

ESP32 -> SSD1306 (OLED)
-----------------------
G21 (Pin 21) -> SDA
G22 (Pin 22) -> SCL
GND          -> GND
3.3V         -> VDD

ESP32 -> SD Card Module
----------------------
G23 (Pin 23) -> MOSI
G19 (Pin 19) <- MISO
G18 (Pin 18) -> SCK
G5  (Pin 5)  -> CS
GND          -> GND
5V         -> VDD 
ESP32 -> Buttons
---------------
G13 (Pin 13) <- RECORD/STOP (pulled up, GND when pressed)
G12 (Pin 12) <- PLAY/PAUSE  (pullسed up, GND when pressed)
G14 (Pin 15) <- NEXT        (pulled up, GND when pressed)

## Notes for ESP32-WROOM-32:
1. This diagram uses G## pin labels as found on your specific board
2. All pins used are safe to use and not dedicated to special functions
3. Ensure your SD card module is 3.3V compatible (or has level shifters)
4. For MAX98357 DAC: GAIN pin can be left disconnected for default gain (15dB)
5. Make sure your buttons are connected to GND when pressed (internal pullups are enabled)
6. SPI (SD card) and I2C (OLED) can run simultaneously without conflicts 
