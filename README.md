# BWCaR107
IDP project for team 107
## Overview of files:
- Daedalus\main.py is main server program to run on PC
- Daedalus\StreamHandlers.py runs asynchronous streams of Arduino data and camera data
- Theseus\theseus.ino runs on the Arduino and is the main controller program, takes input from Daedalus over WiFi
- Theseus\z_secrets.h is a dummy(ish) header file that **MUST** be alphabetically after theseus.ino (for reasons beyond my understanding)
