// vim:ts=4:sw=4:tw=80:et
/*
  sender.ino

  Send packets using rflink library and displays sending results.
  Tested with board "Arduino nano" with CC1101 (TI (tm)) RF device.

  1. LIBRARIES
  ============

  sender.ino and receiver.ino need 3 mandatory libraries:
    cc1101.cpp of package arduino-cc1101, found here:
               https://github.com/veonik/arduino-cc1101
    cc1101wrapper.cpp
    rflink.cpp

  2. CC1101 PLUGGING
  ==================

  The CC1101 device must be plugged as follows:
    CC1101      Arduino nano
    ------      ----
    1 VCC       3.3V
    2 GND       GND
    3 MOSI      D11
    4 SCK       D13
    5 MISO      D12
    6 GDO2      D03  ** On numerous schemas found on Internet, this is plugged
                     ** on D02. This is not cc1101' lib assumption. CC1101'
                     ** lib assumption is GDO2 on D03 and GDO0 on D02.
    7 GDO0      D02  ** SAME REMARK - GDO0 MUST BE PLUGGED ON D02
    8 CSN (SS)  D10
*/

/*
  Copyright 2020 Sébastien Millet

  sender.ino is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  sender.ino is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses>.
*/

#include <Arduino.h>
#include "cc1101wrapper.h"

#define MYADDR       0x0B
#define TARGETADDR   0x5E
#define TXPOWER         0  // 0 = low power, 1 = high power (= long distance)

static RFLink rf;

void setup() {
    Serial.begin(9600);
    cc1101_attach(&rf);
    rf.set_opt_byte(OPT_ADDRESS, MYADDR);
    rf.set_opt_byte(OPT_EMISSION_POWER, TXPOWER);
    Serial.println("Device initialized");
}

const char* msg = "Bonjour chez vous";

void loop() {
    byte n;
    byte r = rf.send(TARGETADDR, msg, strlen(msg) + 1, true, &n);
    if (r != ERR_OK) {
        Serial.print("Error sending message: ");
        Serial.print(rf.get_err_string(r));
        Serial.println("");
    } else {
        Serial.println("Message send successful (received ACK)");
    }
    rf.delay_ms(5000);
}

