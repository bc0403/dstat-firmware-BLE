# dstat-firmware-BLE
Added Bluetooth Low Energy (BLE) connection to the fantastic DStat potentiostat instrument.

## Versions
- V0.1 (April 5, 2017), draft version

## Features
- using BLE or usb to control the potentiostat, the swith can be set by a flag in the codes. We will add a physical swith in future for easy usage.

## Usage
- set the "USBFLAG" in "usartC0.h" for choosing BLE or usb as the control method. 0 means BLE, and 1 means usb.
- the protocol of communication is exactly same as the original DStat.
- verified by "LinkSprite BLE4.0 Shield" (based on TICC2541 ) and "Serial Bluetooth Terminal 1.4" (App from Google Play)


## Hardware connection
- ATxmega256A3U pins <--->   BLE module pins
- Rx (PC2)    <--->   Tx
- Tx (PC3)    <--->   Rx
- VCC         <--->   VCC
- GND         <--->   GND


# Following is DStat documentation
forked from http://microfluidics.utoronto.ca/gitlab/dstat/dstat-firmware

This is the documentation for the DStat microcontroller firmware.

## Table of Contents:

* [Introduction](#introduction)
* [Building from source](#building-from-source)
    * [Build requirements](#build-requirements)
    * [Building with Make](#building-with-make)
* [Installation](#installation)
	* [Requirements](#requirements)
	* [Fuse bits](#fuse-bits)
	* [Firmware files](#firmware-files)
		* [Using Make](#using-make)
		* [Atmel Studio 6](#atmel-studio-6)
* [Connection and testing](#connection-and-testing)
    * [Mac OS X and Linux](#mac-os-x-and-linux)
	* [Windows](#windows)
* [License](#license)

# Introduction
The DStat's operation is controlled by an Atmel XMega microcontroller.
Its main functions are to communicate instructions and data with a computer over a USB CDC interface and perform experiments by controlling the onboard peripherals.
The software that runs on the microcontroller is known as firmware and is written in plain C using many parts of Atmel's ASF framework.

# Building from source
Precompiled binaries of the firmware are [available](http://microfluidics.utoronto.ca/gitlab/dstat/dstat-firmware/builds) from the automated build system (click the download button for the desired build) and can be uploaded using [avrdude](http://www.nongnu.org/avrdude/) or Atmel Studio.
The most up to date firmware can be found in the `develop` (or individual feature) branches.

## Build requirements

* Mac or Linux:
	* GNU Make (OS X: included with Developer Tools)
	* avr-gcc (OS X: install with homebrew)
	* avr-libc (OS X: install with homebrew)
* Windows:
    * Docker

## Building with Make

The easiest way to build the firmware on Mac or Linux (Make is available in Windows, but the Makefiles may need changes to run correctly) is using the included Makefiles (from the root of dstat-firmware):
````
    make
````

This will produce the firmware binaries that will be loaded onto the microcontroller.
`atxmega256a3u_104.hex` is a bootloader for upgrading firmware over USB after the initial flash (currently experimental) and `dstat-firmware.hex` or `dstat-firmware.elf` is the actual firmware.

To compile for a dstat-hardware version earlier than 1.2.3, instead do:
````
    make BOARD_VER_MICRO=0
````

## Building with Make (using Docker)

On Windows, the easiest way to compile the source is using make from docker (Keep in mind you can use the automated build system if you don't intend to make modifications to the firmware).:

1. Install docker from https://docs.docker.com/docker-for-windows/
2. Use the tray icon for Docker to enable sharing on your C drive (under Shared Drives)
3. In a command prompt: `docker run --rm -w /src -v c:/Users/mdryden/src/dstat-firmware/DSTAT:/src --entrypoint make vyivanov/avr-docker`
   Replacing `c:/Users/mdryden/src/dstat-firmware/DSTAT` with the path to wherever you cloned the source.
   This should compile your source files and you can upload them with avrdude or Atmel Studio.

To compile for a dstat-hardware version earlier than 1.2.3, instead do:
````
    docker run --rm -w /src -v c:/Users/mdryden/src/dstat-firmware/DSTAT:/src --entrypoint "make BOARD_VER_MICRO=0" vyivanov/avr-docker
````

# Installation
## Requirements
To install the firmware onto the DStat's microcontroller, you'll need a programming tool that supports PDI.
A simple and inexpensive programmer is the [Olimex AVR ISP Mk2](https://www.olimex.com/Products/AVR/Programmers/AVR-ISP-MK2/open-source-hardware) (available from Digikey and Mouser).
In the near future, updating the firmware over USB will be supported, but a programming tool must still be used at least once to bootstrap the microcontroller.

To load the firmware, you'll need either [Atmel Studio](http://www.atmel.com/tools/atmelstudio.aspx), a full-featured development platform (Windows-only), or [AVRDUDE](http://www.nongnu.org/avrdude/), a cross-platform command line tool.
On Mac OS X, AVRDUDE can be installed using [homebrew](http://brew.sh): `brew install avrdude`

## Fuse bits
The XMega has one of the SPI ports we need configured for JTAG use, by default, and needs to be disabled by setting one of the microcontroller's fuse bits.
The JTAGEN bit of FUSEBYTE4 should be set to 1.
This only needs to be done once per microcontroller.
This must be done manually if you're using Atmel Studio, but will be taken care of automatically by the makefile method.
This can be done from the [Fuses](http://www.atmel.com/webdoc/atmelstudio/atmelstudio.programmingdialog.fuses.html) page of the Device Programming window—simply uncheck `JTAGEN` and click `Program`

## Firmware files
The memory files used to program the microcontroller are found in the root directory:
* Firmware: `dstat-firmware.elf`

Before programming, make sure your programmer is connected to your computer and its PDI connector is connected to the pin header labelled AVR-PDI on the DStat.
The side with the red wire should face the pin labelled 1 on the PCB.

### Using Make
Similar to building from source, the make command can be used to program the microcontroller.
By default, the Makefiles are configured to use an AVR ISP mk2 connected over USB.
If you are using a different programmer, be sure to set the `AVRDUDE_PROG` and `AVRDUDE_PORT` options in the `config.mk` files found in the DSTAT and EEPROM init directories.
See `man avrdude` for more information about these options.

To upload the firmware file and set fuse bits:
````
	make program
````
Occasionally, setting the fuse bits may fail.
If an error occurs on verification, simply run `make program` again.


### Atmel Studio 6
Follow the directions [here](http://www.atmel.com/webdoc/atmelstudio/atmelstudio.AVRStudio.ProgrammingDialog.Introduction.html) to program the microcontroller.
Be sure to select `ATXMEGA256A3U` as the Device and `PDI` as Interface.

In the Memories tab, `dstat-firmware.elf` should be selected for Flash

# Connection and testing
If the firmware has been successfully applied, it will be possible to connect to the DStat over USB.


## Mac OS X and Linux
Mac OS X and Linux do not require driver files and will automatically enumerate the DStat and give it an entry in /dev/ as a virtual serial port.
On OS X, the DStat should appear as `cu.usbmodem12...E1`.
On Linux, check the /dev/ directory for new entries after you plug the DStat in.

Once connected, you can test communications using a terminal emulator (following the communications protocol in `dstat-firmware/communication protocol.txt`):
````
    $ screen /dev/cu.usbmodem12...E1
    $ c
    #
    $ k
    #INFO: 30k
````
## Windows
Unlike Mac and Linux, Windows does not automatically load its USB CDC driver and needs a wrapper driver installed.
The process is similar to installing [Arduino drivers](http://www.arduino.cc/en/guide/windows#toc4), but using the driver file included in the `drivers` directory of `dstat-interface`.
Unfortunately, if you are running Windows 8 or above, this requires disabling driver signing enforcement (thank Microsoft), but actually installing Arduino should install suitable drivers.

# License
ASF code is subject to the following copyright terms:
````
	*Copyright (c) 2012 Atmel Corporation. All rights reserved.
	*
	* Redistribution and use in source and binary forms, with or without
	* modification, are permitted provided that the following conditions are met:
	*
	* 1. Redistributions of source code must retain the above copyright notice,
	*    this list of conditions and the following disclaimer.
	*
	* 2. Redistributions in binary form must reproduce the above copyright notice,
	*    this list of conditions and the following disclaimer in the documentation
	*    and/or other materials provided with the distribution.
	*
	* 3. The name of Atmel may not be used to endorse or promote products derived
	*    from this software without specific prior written permission.
	*
	* 4. This software may only be redistributed and used in connection with an
	*    Atmel microcontroller product.
	*
	* THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
	* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
	* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
	* EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
	* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
	* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
	* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
	* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	* POSSIBILITY OF SUCH DAMAGE.
````
All other code is licensed under the [GNU Public License](/LICENSE) v3.
