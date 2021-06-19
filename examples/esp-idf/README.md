# Micro-RTSP Example

This example shows how to use the Micro-RTSP library with the Ai-Thinker ESP32-CAM board using the IoT Development Framework from Espressif. This example takes the WiFi Station example from esp-idf and combines it with Micro-RTSP, a MQTT client, and OTA updating.

This example uses MQTT to allow control of the LED flash on the ESP32-CAM. It is also needed for the OTA updating.
This example uses the "simple_ota" example as the basis for the OTA functionality. It relies on a http server to retrieve the binary file.
This example also uses network configuration tweaks from the iperf example.
This example supports multiple simultaneious TCP and UDP clients. All clients will use the same camera settings. Be aware that performance will be greatly reduced with multiple clients.

This project has been tested with esp-idf version 4.4.

This project epends on the `esp32-camera` library which should be cloned into the `components` folder when this repo is cloned (it is included as a git submodule).

## How to use example

### Configure the project

Open the project configuration menu (`idf.py menuconfig`). 

In the `Example Configuration` menu:

* Set the Wi-Fi configuration.
    * Set `WiFi SSID`.
    * Set `WiFi Password`.
    * Set your desired framerate.
    * Set vertical and horizontal flipping of the image (depends on how the camera is mounted).

Additional configuration options for the camera (resolution, quality, whitebalance) can be found in main/esp32cam.cpp
Configuration options for the MQTT client can be found in main/esp32cam.cpp

### Build and Flash

Build the project and flash it to the board, then run the monitor tool to view the serial output:

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

Once the project is flashed to the board, you can use the ota_update.py python script to update the ESP32-CAM once it is connected to your WiFi and your MQTT broker.

To update OTA, run `python ota_update.py <SERVER_PORT> <LWIP_LOCAL_HOSTNAME>` where `<SERVER_PORT>` is what to use for the local HTTP server (8443 works fine) and `<LWIP_LOCAL_HOSTNAME>` is the device hostname set in menuconfig.
Example: `python ota_update.py 8443 esp32cam1`

See the Getting Started Guide for all the steps to configure and use the ESP-IDF to build projects.

* [ESP-IDF Getting Started Guide on ESP32](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)
* [ESP-IDF Getting Started Guide on ESP32-S2](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html)
* [ESP-IDF Getting Started Guide on ESP32-C3](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/index.html)


