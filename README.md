# TT7-40-Tracker
Eagle files and code for TT7-40 high altitude balloon tracker.

The board consists of an ATMEGA328P running at 4MHz, an RFM22B transmitter and a UBLOX MAX7-C GPS module.
The code transmits a telemetry string via RTTY. (434.301MHz, 100 baud, 500Hz shift, 2 stop bits, ASCII 7)
