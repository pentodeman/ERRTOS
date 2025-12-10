## ECE 4380-001 Final Project: Group 26
- Rexford Brittingham
- Dominik Kuzniarz
- Quinton Cook

## Hardware
- Main system features:
  - Switching two independent loads
  - Measuring current through the loads
  - Measuring mains voltage
  - Displaying power consumption of the two loads on a display
  - Logging power consumption data to a web interface
  - Switching Hardware Powered Over USB-C PD
- System diagram:
<img src="Embedded_Project_Hardware.drawio.svg" />

SVG file: /Embedded_Project_Hardware.drawio.svg

## Embedded Device
- Raspberry Pi Pico 2 W:
  - 32 Bit SBC
  - WiFi Chip Included for Easy Connection to UI Webserver
  - Compatible with Arduino Libraries through the Arduino-Pico Library
  - Programmable Through Arduino IDE (see Arduino-Pico-Sketch) Over USB Micro-B

## Libraries/Dependencies
- Pi Pico Programming:
  - Arduino GFX Library (For output to built-in display)
  - ArduinoHttpClient (For Sending Sampled Data and Receiving Control Commands with Webserver)
  - Arduino.h (Basic Embedded System Functions/APIs)
  - Arduino-Pico WiFi.h (Interfacing with the Pi Pico 2 W's Included 802.11 Radio Hardware)
- UI Webserver:
  - fastapi
  - uvicorn
  - jinja
  - python websockets
  - python wsproto

## Starter Guide
- Hardware Creation/Construction Not Covered, refer to the system diagram
- Packages for UI Webserver mentioned above must be installed via system or pip
- Arduino IDE must have Additional Board Manager URLs:
  - https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json 
  - https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
  - http://arduino.esp8266.com/stable/package_esp8266com_index.json
- And "Pi Pico Programming" Libraries mentioned above must be installed
- Alternatively, Libraries may be downloaded and linked manually to the user's IDE of choice
- From there, open the included .ino sketch, Verify compilation, and adjust server/client addresses along with network SSID/password as necessary to your LAN
- Once networking details have been adjusted, program to the Pi Pico
- Additionally, run "uvicorn main:app --host 0.0.0.0 --port 8000 --reload" from within the Server directory to begin the webserver
- From there, the uvicorn terminal should provide a HTTP URL for the frontend Web UI, and the Pi, if powered, should be reporting back to it with further data
- Turning the load outlets on/off from the UI may take some time to register, but should result in changes to displayed states and an audible click from the relays
