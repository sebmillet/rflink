// vim:ts=4:sw=4:tw=80:et
/*
  receiver2.ino

  Receive packets using CC1101 device plugged on Arduino and display some debug
  data along with receptions.
  Tested with board "Arduino nano"

  See sender.ino in the examples/sender directory about libraries and CC1101
  plugging.
*/

/*
  Copyright 2020 Sébastien Millet

  receiver.ino is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  receiver.ino is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses>.
*/

#include <Arduino.h>
#include "cc1101wrapper.h"

#define MYADDR       0xCE
#define TXPOWER         0  // 0 = low power, 1 = high power (= long distance)

static RFLink rf;

void serial_printf(const char* fmt, ...)
     __attribute__((format(printf, 1, 2)));

void serial_printf(const char *fmt, ...) {
    char buffer[150];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    buffer[sizeof(buffer) - 1] = '\0';
    Serial.print(buffer);
}

static void serial_begin(long speed) {
    Serial.begin(speed);
}

void setup() {
    serial_begin(9600);
    cc1101_attach(&rf);
    rf.set_opt_byte(OPT_ADDRESS, MYADDR);
    rf.set_opt_byte(OPT_EMISSION_POWER, TXPOWER);
    serial_printf("Device initialized\n");
}

char buffer[50];

void loop() {
    byte len, sender, r;
    if ((r = rf.receive(&buffer, sizeof(buffer), &len, &sender)) != ERR_OK) {
        serial_printf("Reception error: %s\n", rf.get_err_string(r));
    } else {
        if (len >= sizeof(buffer))
            len = sizeof(buffer) - 1;
        buffer[len] = '\0';  // In case received message would be not-a-string
        serial_printf("Received from 0x%02x: '%s'\n", sender, buffer);
    }
}

