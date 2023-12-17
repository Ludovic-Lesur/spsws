# Description

The aim of the **MeteoFox** project was to design an autonomous weather station using solar energy and Sigfox connectivity. The **SPSWS** is the main processing board of the device.

# Hardware

The boards were designed on **Circuit Maker V1.3**. Below is the list of hardware revisions:

| Hardware revision | Description | Status |
|:---:|:---:|:---:|
| [SPSWS HW1.0](https://365.altium.com/files/C5470066-C92D-11EB-A2F6-0A0ABF5AFC1B) | Initial version for prototyping. | :x: |
| [SPSWS HW2.0](https://365.altium.com/files/C7B06FC0-C92D-11EB-A2F6-0A0ABF5AFC1B) | PCB fitted to outdoor casing.<br>Add connectors to external sensors and antennas.<br>Separate SPI interfaces for radio and ADC.<br>Improved power tree. | :white_check_mark: |

# Embedded software

## Environment

The embedded software was developed under **Eclipse IDE** version 2019-06 (4.12.0) and **GNU MCU** plugin. The `script` folder contains Eclipse run/debug configuration files and **JLink** scripts to flash the MCU.

## Target

The SPSWS boards are based on the **STM32L041K6U6** (HW1.0) and the **STM32L081C8T6** microcontroller of the STMicroelectronics L0 family. Each hardware revision has a corresponding **build configuration** in the Eclipse project, which sets up the code for the selected board version.

## Structure

The project is organized as follow:

* `inc` and `src`: **source code** split in 6 layers:
    * `registers`: MCU **registers** adress definition.
    * `peripherals`: internal MCU **peripherals** drivers.
    * `utils`: **utility** functions.
    * `components`: external **components** drivers.
    * `sigfox`: **Sigfox EP library** low level implementation.
    * `applicative`: high-level **application** layers.
* `lib`: **Sigfox EP_LIB and ADDON_RFP** submodules.
* `startup`: MCU **startup** code (from ARM).
* `linker`: MCU **linker** script (from ARM).

## Sigfox library

**Sigfox technology** is very well suited for this application for 3 main reasons:

* **Data quantity is low**, weather data can be packaged on a few bytes and does not require high speed transmission.
* **Low power** communication enables **energy harvesting** (solar cell + supercap in this case), so that the device is autonomous.
* The weather station can be placed is very isolated places thanks to the **long range** performance.

The project is based on the [Sigfox end-point open source library](https://github.com/sigfox-tech-radio/sigfox-ep-lib) which is embedded as a **Git submodule**.
