// vim:ts=4:sw=4:tw=80:et
/*
  cc1101wrapper.cpp

  Intermediate code to make work together:
    arduino-cc1101 library published on GitHub under veonik, url:
      https://github.com/veonik/arduino-cc1101
  and
    rflink library

  NOTE

    This is NOT a 'driver' code for CC1101. The driver code is contained in
    arduino-cc1101 library (early 2020: cc1101.h, cc1101.cpp, ccpacket.h).

    The current file is just a quick 'helper' file, that aims to:
    - Avoid any CC1101-specific code inside rflink.{h, cpp}
    - Avoid (as much as possible) too much CC1101-specific code in the code that
      uses rflink
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

#include "cc1101wrapper.h"
#include "rflink.h"

#include <cc1101.h>
#include <ccpacket.h>

//#define CC1101WRAPPER_DEBUG

#ifndef CC1101WRAPPER_DEBUG

#define dbg(a)
#define dbgf(...)
#define dbgbin(a, b, c)

#else

#include "debug.h"

#endif

CC1101 radio;
byte syncWord[2] = {0xA9, 0x5A};

void cc1101_init(byte* max_data_len, bool reset_first) {
    if (reset_first)
        radio.reset();
    radio.init();
    radio.setSyncWord(syncWord);
    radio.setCarrierFreq(CFREQ_868);
    radio.enableAddressCheck();
    *max_data_len = (CCPACKET_DATA_LEN);
}

void cc1101_set_opt(opt_t opt, void *data, byte len) {
    if (opt == OPT_ADDRESS && len == 1) {
        // Set device address
        byte addr = *(byte*)data;
        radio.setDevAddress(addr);
        dbgf("Set device address to: 0x%02x", addr);

    } else if (opt == OPT_EMISSION_POWER && len == 1) {
        // Set power of device, when sending data
        // The data passed in must be a byte, where:
        //   a zero means low power
        //   any non zero means high power (a.k.a "long distance")
        byte level = *(byte*)data;
        byte pa_value;
        if (!level) {
            pa_value = PA_LowPower;
            dbg("Set device PA to low power");
        } else {
            pa_value = PA_LongDistance;
            dbg("Set device PA to high power");
        }
        radio.setTxPowerAmp(pa_value);

    } else if (opt == OPT_SNIF_MODE && len == 1) {
        byte val = *(byte*)data;
        if (val) {
            radio.disableAddressCheck();
            dbg("Disabled address check (a.k.a. snif mode)");
        } else {
            radio.enableAddressCheck();
            dbg("Enabled address check (a.k.a. non-snif mode)");
        }

    } else {
        dbgf("Error: unknown option code: %i", opt);
    }
}

// FIXME
// cc1101wrapper copies data from caller before handing over to
// sendData method of radio object.
// Using user data directly would be best.
byte cc1101_send(const void *data, byte len) {
    CCPACKET packet;
    packet.length = len;
    memcpy(packet.data, data, len);

    dbgf("cc1101_send: sending packet of %i byte(s):", len);
    dbgbin("cc1101_send:   ", packet.data, len);
    bool r = radio.sendData(packet);
    dbgf("cc1101_send: sendData return value: %i", r);

    return r ? ERR_OK : ERR_SEND_IO;
}

// FIXME
// Same remark as with cc1101_send: a lot of memcpy in the end, in the
// way it is designed today.
byte cc1101_receive(void *buf, byte buf_len) {
    CCPACKET packet;
    byte len;
    if ((len = radio.receiveData(&packet))) {

        dbgf("cc1101_receive: %i byte(s) packet received:", len);
        dbgbin("cc1101_receive:   ", packet.data, len);

        if (len > buf_len)
            len = buf_len;

        memcpy(buf, packet.data, len);
        return len;
    } else {
        return 0;
    }
}

void cc1101_set_interrupt(void (*func)()) {
    attachInterrupt(CC1101Interrupt, func, FALLING);
}

void cc1101_reset_interrupt() {
    detachInterrupt(CC1101Interrupt);
}

void cc1101_attach(RFLink* link) {
    RFLinkFunctions f;
    f.deviceInit = cc1101_init;
    f.deviceSend = cc1101_send;
    f.deviceReceive = cc1101_receive;
    f.deviceSetOpt = cc1101_set_opt;

    f.setInterrupt = cc1101_set_interrupt;
    f.resetInterrupt = cc1101_reset_interrupt;

    link->register_funcs(&f);
}

