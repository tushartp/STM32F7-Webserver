# STM32F7-Webserver


This is the secure webserver demo running on STM32F769I-DISCO board from STMicroelectronics

This demo uses webserver library from https://libwebsockets.org/

It contains the stable version of library from v3.1-stable branch from https://github.com/warmcat/libwebsockets/tree/v3.1-stable and some modification needed to work with FatFs, LWIP and FreeRTOS.

To compile the project please download free eclipse based IDE from ST's website https://www.st.com/en/development-tools/stm32cubeide.html


Prepare a SDcard and copy the contents of SDCARD folder on to the SDcard directly to have the webpages loaded from it. Insert the SD card in the uSD card slot on the discovery board.
Compile the project and debug and run using ST-Link or J_link debugger.
