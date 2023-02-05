# Summary
The aim of the MeteoFox project was to design an autonomous weather station, using solar energy and Sigfox connectivity. The SPSWS is the main processing board of the device.

# Hardware
The board was designed on **Circuit Maker V1.3**. Hardware documentation and design files are available @ https://circuitmaker.com/Projects/Details/Ludovic-Lesur/SPSWSHW2-0

# Embedded software

## Environment
The embedded software was developed under **Eclipse IDE** version 2019-06 (4.12.0) and **GNU MCU** plugin. The `script` folder contains Eclipse run/debug configuration files and **JLink** scripts to flash the MCU.

## Target
The SPSWS board is based on STMicroelectronics L0 family microcontrollers:
* **STM32L041K6U6** on HW1.0 revision.
* **STM32L081C8T6** on HW2.0 revision.

Each hardware revision has a corresponding **build configuration** in the Eclipse project, which sets up the code for the selected target.

## Structure
The project is organized as follow:
* `inc` and `src`: **source code** split in 6 layers:
    * `registers`: MCU **registers** adress definition.
    * `peripherals`: internal MCU **peripherals** drivers.
    * `utils`: **utility** functions.
    * `components`: external **components** drivers.
    * `sigfox`: **Sigfox library** API and low level implementation.
    * `applicative`: high-level **application** layers.
* `lib`: **Sigfox protocol library** files.
* `startup`: MCU **startup** code (from ARM).
* `linker`: MCU **linker** script (from ARM).

## Sigfox library

Sigfox technology is very well suited for this application for 3 main reasons:
* Data quantity is low, weather data can be packaged on a few bytes and does not require high speed transmission.
* Low power communication enables energy harvesting (solar cell + supercap in this case), so that the device is autonomous.
* The weather station can be placed is very isolated places thanks to the long range performance.

The Sigfox library is a compiled middleware which implements Sigfox protocol regarding framing, timing and RF frequency computation. It is based on low level drivers which depends on the hardware architecture (MCU and transceiver). Once implemented, the high level API exposes a simple interface to send messages over Sigfox network.

Last version of Sigfox library can be downloaded @ https://build.sigfox.com/sigfox-library-for-devices

For this project, the Cortex-M0+ version compiled with GCC is used.
