// vim:ts=4:sw=4:tw=80:et
/*
  cc1101wrapper.h

  Header file of cc1101wrapper.cpp
*/

/*
  Copyright 2020 Sébastien Millet

  rflink is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.

  rflink is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this program. If not, see
  <https://www.gnu.org/licenses>.
*/

#ifndef _CC1101WRAPPER_H
#define _CC1101WRAPPER_H

#include "rflink.h"

#include <Arduino.h>

// Attach CC1101 pins to their corresponding SPI pins
// Uno or Nano pins:
//   CSN  (SS)   => 10
//   MOSI (SI)   => 11
//   MISO (SO)   => 12
//   SCK  (SCLK) => 13
//   GD0  (GDO0) => A valid interrupt pin for your platform (defined below this)
//                  Typically: 2
//   GD2  (GDO2) => In the set {2, 3}, the one not used by GDO0.
//                  Typically: 3
//
// * ************** **********************************************************
// * VERY IMPORTANT **********************************************************
// * ************** **********************************************************
// *
// * ON MANY SCHEMATICS FOUND ON INTERNET, GDO0 IS CONNECTED ON ARDUINO' D3.
// * THIS IS NOT COMPATIBLE WITH CC1101 DEFAULT LIB SETTINGS, THAT ASSUMES GDO0
// * IS CONNECTED ON D2.
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define CC1101Interrupt 4 // Pin 19
#define CC1101_GDO0 19
#elif defined(__MK64FX512__)
// Teensy 3.5
#define CC1101Interrupt 9 // Pin 9
#define CC1101_GDO0 9
#else
#define CC1101Interrupt 0 // Pin 2
#define CC1101_GDO0 2
#endif

void cc1101_attach(RFLink* link);

#endif // _CC1101WRAPPER_H

