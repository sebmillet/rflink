RFLink
======

This is an Arduino library for managing level 2 ("link") communication with RF
devices.
Tested and used with Texas Instruments (tm) CC1101 device.
Works on top of arduino-CC1101 library (see below).

The link layer implements:
  - Packet ID, to properly ignore already-received packets
  - ACK, so that the sender will know data good reception

The notion of sender and receiver address is managed by RFLink, although this
is in part already managed as a built-in feature of CC1101.


Installation
------------

Download a zip of this repository, then include it from the Arduino IDE.

**IMPORTANT**
You'll also need to install arduino-cc1101 library, available here:
  [https://github.com/veonik/arduino-cc1101](https://github.com/veonik/arduino-cc1101)


Usage
-----

See [examples/sender/sender.ino](examples/sender/sender.ino) and
[examples/receiver/receiver.ino](examples/receiver/receiver.ino) for an
example.

