# Bluetooth Remote Controller for PC

This repository contains code for a Bluetooth Low Energy (BLE) remote controller for PC using an ESP32 microcontroller as the server and a Python script as the client. The remote controller utilizes a 4x4 button matrix to send commands to the PC.

## Components

`controller.c` : This file contains the code for the ESP32 microcontroller acting as the BLE server. It manages the connection with the client and interprets the button presses from the 4x4 matrix to send corresponding commands to the PC.

`client.py` : This Python script serves as the client running on the PC. It establishes a connection with the ESP32 server via BLE and listens for commands sent by the controller. Upon receiving commands, it executes the appropriate actions on the PC.

## How to Use

  ### Hardware Setup:
  Connect the ESP32 microcontroller to the 4x4 button matrix according to the wiring diagram provided in the documentation (GPIO pins are recommended in the controller.c).
  Ensure that the ESP32 is powered and properly configured.

  ### Software Setup:
  Flash the controller.c file onto the ESP32 microcontroller using Espressif-IDE. Don't forget to config in the menuconfig to use BLE and NVS.
  Run the client.py script on your PC. Make sure you have Python installed and the necessary dependencies (such as Bleak and more) are installed.

  ### Usage:
  Once both the server (ESP32) and client (PC) are set up, pair the controller to the PC using the PCs bluetooth settings, and then start the client.
  Press the buttons on the 4x4 matrix to send commands to the PC. The controller.c file maps each button to a specific command.
  The client.py script receives these commands and executes the corresponding actions on the PC.

## Contributing

Contributions to improve this project are welcome! If you have any suggestions, bug fixes, or enhancements, feel free to open an issue or submit a pull request.
## License

This project is licensed under the MIT License. Feel free to use and modify the code as per the terms of the license.
