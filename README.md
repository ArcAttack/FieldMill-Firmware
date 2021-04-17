# FieldMill-Firmware
Firmware for our FieldMill project

# To install it (Visual studio Code with PlatformIO and the Espressif IDF are required)

1.  Clone or downlaod and extract this repo.
2.  Open the project
3.  Use the "esp32dev -> Platform -> Build Filesystem Image" and "esp32dev -> Platform -> Upload Filesystem Image" to write the website code to the ESP
4.  Upload the firmware using the "esp32dev -> General -> Upload" option

# How to use it

### First steps
The ESP32 will open an unsecured network calle Field Mill for configuration. Connect to it and go to 192.168.4.1 , which will show the current sensor reading.

### Setup
To change settings like wifi and mqtt go to Setup -> Settings. If you change the wifi settings the mill will reboot for them to take effect, if it fails to connect it will re-open its own network.

### Calibration
The mill comes with a default calibration that I created with my calibration setup. If you want the readings to be super accurate you'll need to calibrate it again yourself, which you can only do if you can create a known reference field.
For this you must be connected to a wifi network that has internet access and go to Setup -> Calibration. Then hit the Clear button underneath the graph to clear all exsisting data and start from scratch.

Apply a field and write the strength you applied into the Applied field spinner and click add (make sure to wait 5-10sec for the value to settle!). Do this for as many points as you want to, I did 10 positive and 10 negative plus a zero.

Hit save to store the calibration to the ESP **(YOUR DATA WILL BE LOST IF YOU DON'T DO THIS)**
