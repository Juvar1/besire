# BeSiRe - Better Si4735 Receiver

This DIY radio receiver works with VHF/HF/MF/LF frequency range with following modulations FM/AM/SSB (USB/LSB). FM in VHF only. It combines all the best parts of open source circuit designs for Si4735 chip. And it's user interface is 3.2 inch touchscreen (Adafruit 1743). It uses only one simple telescopic antenna for all frequencies. It has support for stereo sound if available and support for RDS (Radio Data System) program information display if available.

https://www.adafruit.com/product/1743

User can freely change all key parameters with touchscreen by touching a buttons. Available options are volume, frequency, band, modulation, bandwidth and frequency stepping. It has also standby-mode which is activated by first minimizing volume to zero and then touching a off-button which comes visible in place of volume down button. Receiver turns back on when user taps anywhere on the touchscreen. There is also a channel seek up/down command which is activated by long pressing frequency up or down-button accordingly.

Before using SSB feature user must upload SSB-patch into external EEPROM chip. Instructions for that are in link below.

https://github.com/pu2clr/SI4735

Printed circuit board and battery holder fits in to Hammond 1599HBAT enclosure.

This receiver design and it source code is completely open source with GNU GPL license. You can freely use it commercially but limiting only by it's third party libraries licenses.
