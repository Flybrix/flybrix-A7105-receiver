# flybrix-A7105-receiver

Receives a7105 data and generates 6 channel cPPM output using an ATTINY85.

Uses AVR assembly to bit-bang single wire SK6812/WS2811 LED signalling protocol.

Instructions to add ATTINY85 to the Arduino IDE are online here:

http://highlowtech.org/?p=1695

The process involves adding the address "https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json" to the board manager.