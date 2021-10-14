# ESP32 with wifi and ECG
This readme file describes  project firmware and gives the guides on how to use it.
#Build note
Project created using make build system.

# Description

This project use the esp32 board to run the wifi as a station and as a hotspot, depeneds on how it was configurated at start. It looks for a connection, an if it was reached, it get's the configuration of the sensors from the JSON file via wifi. After the configuration is read - system initialize the MAX86150 ECG sensor to use it with required in JSON file parameters.

Also the project has a console component to control each of the connected to the ESP32 board GPIO pins, it could be used 
for pin testing and setting in time of the board hardware and software testing.

# Componenets(modules) used

1. Station configuration module
2. AP configuration module
3. AP_STA configuration module
4. File utils (working with JSON)
5. i2c module to work with MAX86150 ECG sensor
6. HTTP client module sending HTTP requests to the server and working with JSONs.
7. GPIO control module (also used with console)
8. Console component (used for testing the GPIOs, could be upgraded to test each of the modules)
9. MAX86150 driver adapted for esp-idf

# Tutorials
1. Environment(ESP_IDF) Start: https://dl.espressif.com/doc/esp-idf/latest/get-started/index.html#get-started-get-esp-idf
2. Standard Setup of Toolchain for Windows: https://dl.espressif.com/doc/esp-idf/latest/get-started/windows-setup.html
3. Add IDF_PATH to User Profile: https://dl.espressif.com/doc/esp-idf/latest/get-started/add-idf_path-to-profile.html
4. Build and Flash with Eclipse IDE: https://dl.espressif.com/doc/esp-idf/latest/get-started/eclipse-setup.html
5. Eclipse IDE on Windows: https://dl.espressif.com/doc/esp-idf/latest/get-started/eclipse-setup-windows.html#eclipse-windows-setup
6. esp-idf-template project on github: https://github.com/espressif/esp-idf-template

Book ESP32 link: https://leanpub.com/kolban-ESP32

Kconfig language link: https://www.kernel.org/doc/Documentation/kbuild/kconfig-language.txt

# SDK version
ESP-IDF v4.2-dirty

Guide: To check ESP-IDF version run commmand: idf.py --version



# Board information
Current commit: 8c5baeaa76ba362691e375b1d5a41dc3020638f3 (28.04.2021)

Project firmware tested on evaluation kit: ESP-Wrover-Kit v4.1
