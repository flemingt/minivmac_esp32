This is a Mini vMac port for the ESP32-S3, adapted in this working copy for the Guition ESP32-S3-4848S040 4-inch 480x480 panel. The panel uses an ST7701 RGB display and GT911 capacitive touch controller. The emulator image is scaled to fit the square display, and touch controls the emulated mouse.

Usage:
- The sources must be compiled with the IDF from Espressif (version 5.5). (idf.py build).
- A MacPlus ROM (file name: vMac.ROM) and various disks (disk1.dsk to disk6.dsk) must be placed in the spiffs directory. The ROM is required for the Macintosh to start (firmware). The disks appear on the desktop after the operating system has started. disk1.dsk should, of course, be a bootable MacOS (e.g., System 7.x) startup disk.
- Once the files are in the spiffs directory, everything can be flashed.
- idf.py flash
- The emulator then starts directly.
- For mouse and keyboard support (see next chapter)

Mouse and keyboard:

The GT911 touchscreen provides mouse movement and left-click input. Keyboard input is not implemented in this Guition adaptation. The board's USB-C connector is wired to a CH340 USB-to-serial bridge for programming and power; it is not connected as a USB host port, so a USB hub cannot attach a keyboard or mouse to the ESP32-S3. Use a separate BLE HID implementation or an external input bridge for keyboard support.
