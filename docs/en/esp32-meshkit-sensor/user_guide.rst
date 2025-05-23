ESP32-MeshKit-Sense
======================

:link_to_translation:`zh_CN:[中文]`

Overview
------------

ESP32-MeshKit-Sense is a development board with an ESP32 module at its core. It features peripherals, such as a temperature and humidity sensor, an ambient light sensor, etc. The board can be interfaced with screens. The board is mainly used to detect the current consumption of ESP32 modules in a normal operation state or in sleep mode, when connected to different peripherals.

For more information on ESP32, please refer to `ESP32 Datasheet <https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf>`__.

Block Diagram and PCB Layout
---------------------------------

Block Diagram
+++++++++++++++++

The figure below shows the block diagram of ESP32.

.. figure:: ../../_static/esp32-meshkit-sensor/blockdiagram.png
   :align: center
   :alt: ESP32-MeshKit-Sense
   :figclass: align-center

   ESP32 Block Diagram

PCB Layout
++++++++++++++

The figure below shows the layout of ESP32-MeshKit-Sense PCB.

Functional Descriptions of PCB Layout are shown in the following table:

+-----------------------------------+-----------------------------------+
| PCB Elements                      | Description                       |
+===================================+===================================+
| EXT5V                             | 5 V input from USB                |
+-----------------------------------+-----------------------------------+
| CH5V                              | input from the electrical         |
|                                   | charging chip                     |
+-----------------------------------+-----------------------------------+
| CHVBA                             | output from the electrical        |
|                                   | charging chip                     |
+-----------------------------------+-----------------------------------+
| VBA                               | Connects to the positive          |
|                                   | electrode of the battery          |
+-----------------------------------+-----------------------------------+
| SUFVCC                            | When the switch is toggled to the |
|                                   | ”ON” position, it is connected to |
|                                   | the power input. When the switch  |
|                                   | is toggled to the ”OFF” position, |
|                                   | the power supply is disconnected. |
+-----------------------------------+-----------------------------------+
| DCVCC                             | Input from power management chip  |
|                                   | DC-DC                             |
+-----------------------------------+-----------------------------------+
| 3.3V                              | 3.3 V power output from power     |
|                                   | supply management chip            |
+-----------------------------------+-----------------------------------+
| 3.3V_PER                          | 3.3 V power supply for all        |
|                                   | peripherals                       |
+-----------------------------------+-----------------------------------+
| 3.3V_ESP                          | 3.3 V power supply for all ESP32  |
|                                   | modules                           |
+-----------------------------------+-----------------------------------+
| 3.3V_SEN                          | 3.3 V power supply for the three  |
|                                   | on-board sensors                  |
+-----------------------------------+-----------------------------------+
| 3.3V_SCR                          | 3.3 V power supply for the        |
|                                   | off-board screen                  |
+-----------------------------------+-----------------------------------+
| Charge                            | Battery charging indicator, D5 is |
|                                   | a red light, indicating that      |
|                                   | charging is undergoing; D6 is a   |
|                                   | green light, indicating that      |
|                                   | charging is complete.             |
+-----------------------------------+-----------------------------------+
| Sensor                            | Power indicator, indicating that  |
|                                   | 3.3V_Perip_Sensor is enabled      |
+-----------------------------------+-----------------------------------+
| Screen                            | Power indicator, indicates that   |
|                                   | 3.3V_Perip_Screen is enabled      |
+-----------------------------------+-----------------------------------+
| WiFi / IO15                       | Signal indicator, indicating that |
|                                   | Wi-Fi connection is working       |
|                                   | properly                          |
+-----------------------------------+-----------------------------------+
| Network / IO4                     | Signal indicator, indicating the  |
|                                   | board is properly connected to    |
|                                   | the server                        |
+-----------------------------------+-----------------------------------+

Functional Modules
-----------------------

This chapter mainly introduces each functional module (interface) and the hardware schematics for them.

Power Supply Management Module
+++++++++++++++++++++++++++++++++++

.. _power-supply-management-module-1:

Power Supply Management Module
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The development board can be powered by battery and the AP5056 power supply management chip can be used to charge the battery. The AP5056 is a complete constant current constant voltage linear charger for single cell lithium-ion batteries. It has 4.2 V of preset charge voltage and 1 A of programmable charge current.

When both the USB power supply and the battery power supply are available, the system selection of power supply will be: VBUS is high, Q4 is in cut-off state, VBAT (battery power) is automatically cut off from the system power supply, and the USB supplies power for the system.

The figure below shows the schematics for USB/BAT power supply management.

.. figure:: ../../_static/esp32-meshkit-sensor/BATTERY-POWER.png
   :align: center
   :alt: ESP32-MeshKit-Sense
   :figclass: align-center

   USB/BAT Power Supply Management Schematics

Power Supply Management for Peripherals
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

First of all, the input from the USB or BAT is converted by the power management chip into a 3.3 V voltage to power the circuit. The power management chip on the board is ETA3425, which has an output voltage of 3.3 V and a maximum output current of 600 mA.

The figure below shows the schematics for peripheral power supply.

.. figure:: ../../_static/esp32-meshkit-sensor/peripheral.png
   :align: center
   :alt: ESP32-MeshKit-Sense
   :figclass: align-center

   Peripheral Power Supply Schematics

The main VDD33 circuit has two branches:

- ESP32_VDD33, used to power the ESP32 module module
- VDD33_PeriP, used to power all peripherals.

The connection between them can be controlled via the pin header and jumper cap. The figure below shows the schematics for ESP32_VDD33.

.. figure:: ../../_static/esp32-meshkit-sensor/VDD33.png
   :align: center
   :alt: ESP32-MeshKit-Sense
   :figclass: align-center

   ESP32_VDD33 Schematics

The VDD33_PeriP branch circuit also has two sub-branches

- VDD33_PeriP_Screen, dedicated power supply for the external screen
- VDD33_PeriP_Sensor, power supply for the three sensors

The connection of the two can be controlled by the module GPIO+MOS. The figure below shows the schematics for VDD33_PeriP.

.. figure:: ../../_static/esp32-meshkit-sensor/PeriP.png
   :align: center
   :alt: ESP32-MeshKit-Sense
   :figclass: align-center

   VDD33_PeriP Schematics

Boot & UART
++++++++++++++

The development board is integrated with a PROG Header, which can be connected to a ESP-PROG development board via a cable. Users can then connect the micro USB of the ESP-PROG development board to a PC for ESP32-MeshKit-Sense firmware download and debugging.

The figure below shows the schematics for Boot & UART Circuit.

.. figure:: ../../_static/esp32-meshkit-sensor/UART.png
   :align: center
   :alt: ESP32-MeshKit-Sense
   :figclass: align-center

   Boot & UART Circuit

Module for Wakeup from Sleep
++++++++++++++++++++++++++++++++

The board has a button connected to the pin IO34, which is a pin in the RTC domain. When the chip is in sleep, pressing the button will wake up ESP32.

The figure below shows the schematics for wakeup-from-sleep module.

.. figure:: ../../_static/esp32-meshkit-sensor/wakeup.png
   :align: center
   :alt: ESP32-MeshKit-Sense
   :figclass: align-center

   Wake-from-Sleep Module Schematics

External Screens
++++++++++++++++++++

The development board is integrated with a screen connector that can connect different external screens to the board via cables.

The figure below shows the schematics for external screens.

.. figure:: ../../_static/esp32-meshkit-sensor/screen.png
   :align: center
   :alt: ESP32-MeshKit-Sense
   :figclass: align-center

   Schematics for External Screens

Sensors
++++++++++

Temperature and Humidity Sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The HTS221 is an ultra-compact sensor for relative humidity and temperature. A 3.3 V power supply and I2C interface on the board are dedicated to HTS221.

The figure below shows the schematics for the temperature and humidity sensor.

.. figure:: ../../_static/esp32-meshkit-sensor/THsensor.png
   :align: center
   :alt: ESP32-MeshKit-Sense
   :figclass: align-center

   Temperature and Humidity Sensor Schematics

Ambient Light Sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~

The BH1750FVI is a digital ambient light sensor. A 3.3 V power supply and I2C interface on the board are dedicated to HTS221.

The figure below shows the schematics for the ambient light sensor.

.. figure:: ../../_static/esp32-meshkit-sensor/ambientlightsensor.png
   :align: center
   :alt: ESP32-MeshKit-Sense
   :figclass: align-center

   Ambient Light Sensor Schematics

Ambient Brightness Sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The APDS-9960 is a ambient brightness sensor featuring advanced gesture detection, proximity detection, digital Ambient Light Sense (ALS) and Color Sense (RGBC). It also incorporates an IR LED driver. The development board uses 3.3V power supply and I2C interface. It should be noted that this device is not surface-mounted by default.

The figure below shows the schematics for the ambient brightness sensor.

.. figure:: ../../_static/esp32-meshkit-sensor/proximity.png
   :align: center
   :alt: ESP32-MeshKit-Sense
   :figclass: align-center

   Ambient Brightness Sensor Schematics

Example
------------

See `esp-mdf/examples/development_kit/sense <https://github.com/espressif/esp-mdf/tree/master/examples/development_kit/sense>`__.


Related Documents
--------------------

.. only:: latex

   Please download the following documents from `the HTML version of esp-dev-kits Documentation <https://docs.espressif.com/projects/esp-dev-kits/en/latest/{IDF_TARGET_PATH_NAME}/index.html>`_.

* `ESP32-MeshKit-Sense Schematic`_
* `ESP32-MeshKit-Sense PCB Layout`_

.. _ESP32-MeshKit-Sense Schematic: https://dl.espressif.com/dl/schematics/ESP32-MESHKIT-SENSE_V1_1-0917A.pdf
.. _ESP32-MeshKit-Sense PCB Layout: https://dl.espressif.com/dl/schematics/ESP32-MeshKit-Sense_V1.1.pdf
