# Grids to Teensy

Grids' basic clock frequency is 8 KHz.

Button is debounced at 800 Hz.

"clock reset inputs" are scanned at 8 KHz.
Pots are scanned at 8 KHz.
Pattern Generator is updated at 8 KHz.


## Grids I/O

    HI Inputs:
       1 TAP/reset button
       7 potentiometers
    Signal Inputs:
       1 MIDI in
       1 clock input
       1 reset input
       6 analog inputs
    HI Outputs:
       1 tempo LED
       3 LEDs
    Signal Outputs:
       3 triggers
       3 accents
       1 clock echo
       1 random bit


## Mapping to Teensy 3.X + Audio Board

    Audio Board
      pins 9 11 13 18 19 22 23
    7 pots
      A0 A1 A2 A3 A6 A7 A14 (pins 14 15 16 17 20 21 N/A)
    1 MIDI in
      pin 0
    1 tap/reset button
      pin 2
    1 clock input
      pin 3
    1 reset input
      pin 4
    1 tempo LED
      pin 5
    3 LEDs
      pins 6 7 8
    No trigger outputs
    No accent outputs
    No clock echo
    No random bit
   

## Mapping to Teensy LC

    Digital Inputs
       pin 0 - MIDI in
       pin 2 - tap/reset button
       pin 3 - clock input
       pin 4 - reset input

    Analog Inputs
       pin 14 (A0) - Bass Fill
       pin 15 (A1) - Chaos
       pin 16 (A2) - Snare Fill
       pin 17 (A3) - Y
       pin 21 (A7) - High Hat Fill
       pin 23 (A9) - X
       pin 26 (A12) - Clock

    LED Outputs
       pin 5 - Bass LED
       pin 6 - Snare LED
       pin 7 - High Hat LED
       pin 13 - Tempo LED

    Signal Outputs
       pin 20 - Bass Trigger
       pin 19 - Snare Trigger
       pin 18 - High Hat Trigger
       pin  8 - Bass Accent
       pin  9 - Snare Accent
       pin 10 - High Hat Accent
       pin 11 - clock echo
       pin 12 - random bit
   
## Mapping Grids + Peaks.drums to Teensy 3.X + Audio Board + Breakout

    TBD - at least 13 pots

## Strategies

* build a board on an XL JIGMOD base.
* implement full grids, not peaks, on Teensy LC/Arduino.
* Copy the algorithms and the drum data.

