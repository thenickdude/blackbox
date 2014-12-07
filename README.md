# Blackbox flight data recorder

![Rendered flight log frame](http://i.imgur.com/FBphB8c.jpg)

WARNING - This firmware is experimental, and may cause your craft to suddenly fall out of the sky (though I had no
problems during 60 test flights). No warranty is offered: if your craft breaks, you get to keep both pieces.

## Introduction
This is a modified version of Baseflight for the Naze32 which adds a flight data recorder function ("Blackbox"). Flight
data information is transmitted over the Naze32's serial port on every Baseflight loop iteration to an external logging
device to be recorded.

After your flight, you can process the resulting logs on your computer to either turn them into CSV (comma-separated
values) or render your flight log as a video.

Grab a zip file from the `downloads/` folder for your operating system to get the firmware and tools. If you're
using Linux, you'll need to clone [the repository][] and build them from source instead, instructions are further
down this readme.

[the repository]: https://github.com/thenickdude/blackbox

## Logged data
The blackbox records flight data on every iteration of the flight controller's control loop. It records the current
time in microseconds, P, I and D corrections for each axis, your RC command stick positions (after applying expo
curves), gyroscope data, accelerometer data (after your configured low-pass filtering), and the command being sent to
each motor speed controller. This is all stored without any approximation or loss of precision, so even quite subtle
problems should be detectable from the fight data log.

Currently, the blackbox attempts to log GPS data whenever new GPS data is available, but as I haven't been able to get
my GPS receiver working on my craft, I haven't been able to test it yet. The CSV decoder and video renderer do not yet
show any of the GPS data (though this will be added). If you have a working GPS, please send me your logs so I can get
the decoding implemented.

The data rate for my quadcopter configuration is about 10.25kB/s, with an update rate of 416Hz. This allows me to fit
about 18 days of flight logs on a 16GB MicroSD card, which ought to be enough for anybody :).

## Supported configurations
The flight log data is transferred in flight over the Naze32's main serial port to a data logger. This is the serial
port that connects to the Naze's USB port, the FrSky telemetry port, and the "Rx/Tx" two-pin header in the center of the
board. If you're currently using FrSky telemetry on the FrSky pins, you'll need to move that to one of the [softserial ports][]
instead. This is the serial port normally used for an OSD, so if you have an OSD, you'll need to remove that in order to
use the flight data recorder.

The maximum data rate for the flight log is fairly restricted, so anything that increases the load can cause the flight
log to drop frames and contain errors.

The Blackbox was developed and tested on a quadcopter. It should work on hexacopters or octocopters, but they transmit
more information to the flight log (due to having more motors) so the number of dropped frames may increase. I don't
know if it works on tricopters. The `blackbox_render` tool only supports quadcopters (please send me flight logs from
other craft, and I can add support for them!)

Baseflight's "looptime" setting will decide how many times per second an update is saved to the flight log. The
software was developed on a craft with a looptime of 2400. Any looptime smaller than this will put more strain on the
data rate than I've tested for. A looptime of 0 (i.e. maximum speed) will never work. The default looptime on Baseflight
is 3500.

The Blackbox has only really been tested with the "rate" flight mode. It should continue to record useful data on the
angle or horizon modes, but as I don't fly with those modes, they have had minimal testing.

[Softserial ports]: http://www.netraam.eu/nazewiki/pmwiki.php?n=Howto.FrskyTelemetry

## Hardware
The blackbox software is designed to be used with an [OpenLog serial data logger][] and a microSDHC card. You need a
little prep to get the OpenLog ready for use, so here are the details:

### Firmware
The OpenLog should be flashed with the [OpenLog Lite firmware][] using Arduino IDE in order to minimise dropped frames
(target the "Arduino Uno"). Note that the .hex file currently in the OpenLog repository is out of date with respect to the .ino
source file, use the version I've built in the `tools/blackbox/openlog/` directory instead. Or you can build your own 
.hex file if you add the [required libraries][] to your Arduino libraries directory.

To flash the firmware, you'll need to use an FTDI programmer like the [FTDI Basic Breakout][] along with some way of
switching the Tx and Rx pins over (since the OpenLog has them switched) like the [FTDI crossover][].

[OpenLog serial data logger]: https://www.sparkfun.com/products/9530
[OpenLog Lite firmware]: https://github.com/sparkfun/OpenLog/tree/master/firmware/OpenLog_v3_Light
[Required libraries]: https://code.google.com/p/beta-lib/downloads/detail?name=SerialLoggerBeta20120108.zip&can=4&q=
[FTDI Basic Breakout]: https://www.sparkfun.com/products/9716
[FTDI crossover]: https://www.sparkfun.com/products/10660

### Serial port
Connect the "TX" pin from the two-pin TX/RX header on the centre of the Naze32 to the OpenLog's "RXI" pin. Don't
connect the Naze32's RX pin to the OpenLog.

The OpenLog accepts input power voltages from 3.3 to 12V, so if you're powering the Naze32 with something like 5 volts
from a BEC, you can connect the VCC and GND pins on the OpenLog to one of the Naze32's spare motor headers in order to
power it.

### microSDHC
Your choice of microSDHC card is very important to the performance of the system. The OpenLog relies on being able to
make many small writes to the card with minimal delay, which not every card is good at. A faster SD-card speed rating is
not a guarantee of better performance.

#### microSDHC cards known to have poor performance
 - Generic 4GB Class 4 microSDHC card - the rate of missing frames is about 1%, and is concentrated around the most
   interesting parts of the log!

#### microSDHC cards known to have good performance
 - Transcend 16GB Class 10 UHS-I microSDHC (typical error rate < 0.1%)
 - Sandisk Extreme 16GB Class 10 UHS-I microSDHC (typical error rate < 0.1%)

You should format any card you use with the [SD Association's special formatting tool][] , as it will give the OpenLog
the best chance of writing at high speed. You must format it with either FAT, or with FAT32 (recommended).

[SD Association's special formatting tool]: https://www.sdcard.org/downloads/formatter_4/

### OpenLog configuration
Power up the OpenLog with a microSD card inside, wait 10 seconds or so, then power it down and plug the microSD card
into your computer. You should find a "CONFIG.TXT" file on the card. Edit it in a text editor to set the first number
(baud) to 115200. Set esc# to 0, mode to 0, and echo to 0. Save the file and put the card back into your OpenLog, it
should use those settings from now on.

### Protection
I wrapped my OpenLog in black electrical tape in order to insulate it from my conductive carbon fiber frame, but this
makes its status LEDs impossible to see. I would recommend wrapping it with some clear heatshrink tubing instead.

![OpenLog installed](http://i.imgur.com/jYyZ0oC.jpg "OpenLog installed in my frame with double-sided tape, SDCard slot pointing outward")

## Installation of firmware
Before installing the new firmware onto your Naze32, back up your configuration: Connect to your flight controller
using the [Baseflight Configurator][], open up the CLI tab and enter "dump" into the box at the bottom and press enter. Copy
all of the text that results and paste it into a text document somewhere for safe-keeping.

Click the disconnect button, then on the main page choose the Firmware Flasher option. Tick the box for "Full Chip
Erase" (warning, this will erase all your settings!). Click the "Load firmware (local)" button, and select the file `baseflight_NAZE.hex`
(if you downloaded the source version, it's in the `obj/` directory). Click the "Flash Firmware" button and wait for it
to complete.

Now you need to reload your configuration: Go to the CLI tab and paste in the dump that you saved earlier and press
enter, it should execute and restore your settings.

Before you leave the CLI tab, enable the Blackbox feature by typing in `feature BLACKBOX` and pressing enter. Leave the
CLI tab and your configuration should be saved and the flight controller will reboot. You're ready to go!

If you ever need to disable the Blackbox (say, for example, to switch to using the serial port for an OSD instead), you
can either reflash the stock Baseflight firmware using the Configurator, or you can just turn off the Blackbox feature
by entering `feature -BLACKBOX` on the CLI tab.

[Baseflight Configurator]: https://chrome.google.com/webstore/detail/baseflight-configurator/mppkgnedeapfejgfimkdoninnofofigk?hl=en

## Usage
The Blackbox starts recording data as soon as you arm your craft, and stops when you disarm. Each time the OpenLog is
power-cycled, it begins a fresh new log file. If you arm and disarm several times without cycling the power (recording
several flights), those logs will be combined together into one file. The command line tools will ask you to pick which
one of these flights you want to display/decode.

The OpenLog requires a couple of seconds of delay after powerup before it's ready to record, so don't arm your craft
immediately after connecting the battery (you'll probably be waiting for the Naze to become ready during that time
anyway!)

You should also wait a few seconds after disarming the quad to allow the OpenLog to finish saving its data.

Don't insert or remove the SD card while the OpenLog is powered up.

After your flights, you'll have a series of files labeled "LOG00001.TXT" etc. on the microSD card. You'll need to
decode these with the `blackbox_decode` tool to create a CSV (comma-separated values) file, or render it into a series of
PNG frames with `blackbox_render` which you could convert into a video using another software package.

### Using the blackbox_decode tool
This tool converts a flight log binary file into CSV format. Typical usage (from the command line) would be like:

```bash
blackbox_decode LOG00001.TXT > output.csv
```

Use the `--help` option for more details.

### Using the blackbox_render tool
This tool converts a flight log binary file into a series of transparent PNG images that you could overlay onto your
flight video using a video editor (like [DaVinci Resolve][]). Typical usage (from the command line) would be like:

```bash
blackbox_render LOG00001.TXT
```

This will create PNG files at 30 fps into the same directory as the log file.

Use the `--help` option for more details.

(At least on Windows) if you just want to render a log file using the defaults, you can drag and drop a log onto the
blackbox_render program and it'll start generating the PNGs immediately.

[DaVinci Resolve]: https://www.blackmagicdesign.com/products/davinciresolve

## Building firmware
If you want to rebuild the modified firmware for the Naze32, the procedure is the same as for Baseflight. Once your
[toolchain][] is set up, you merely need to run `make TARGET=NAZE` in the root directory.

If you don't want to rebuild the firmware, that's fine, prebuilt firmware hex files for the Naze32 can be found in the `obj/`
directory.

[toolchain]: https://launchpad.net/gcc-arm-embedded

## Building tools
The `blackbox_decode` tool for turning binary flight logs into CSV doesn't depend on any libraries, so can be built by
entering the `tools/blackbox/` directory and running `make obj/blackbox_decode` . You can add the resulting
blackbox_decode program to your system path to make it easier to run.

The `blackbox_render` tool renders a binary flight log into a series of PNG images which you can overlay on your flight
video. Please read the section below that most closely matches your operating system for instructions on getting the `libcairo`
library required to build the `blackbox_render` tool.

#### Ubuntu
You can get the tools required for building by entering these commands into the terminal:

```bash
sudo apt-get update
sudo apt-get install make gcc libcairo2-dev
```

Build the tool by entering the `tools/blackbox/` directory and typing `make obj/blackbox_render` (or build both tools by
just running `make` ).

#### MacOSX
The easiest way to build is to install the [Xcode development tool][],
then install an environment like [Homebrew][] or [MacPorts][] onto your system.

From MacPorts, you would do this to get LibCairo:

```bash
sudo port selfupdate
sudo port install cairo
```

Afterwards you can run `make` in the `tools/blackbox/` directory to build blackbox_render.

[Xcode development tool]: https://itunes.apple.com/us/app/xcode/id497799835
[Homebrew]: http://brew.sh/
[MacPorts]: https://www.macports.org/

#### Windows
The tools can be built with Visual Studio Express 2013, just open up the solution in the `tools/blackbox/visual-studio/`
folder. You'll need to include the .DLL files from `/tools/blackbox/lib/win32` in the same directory as your built
executable.

## License

This project is licensed under GPLv3.

The binary version of `blackbox_render` for MacOSX is statically linked to these libraries:

 - libbz2 http://www.bzip.org/ (BSD-like)
 - zlib http://www.zlib.net/
 - libcairo & libpixman http://cairographics.org/ (LGPL)
 - libfreetype http://www.freetype.org/ (BSD-like/GPLv2)
 - libpng16 http://www.libpng.org/pub/png/libpng.html
 
The windows binary of `blackbox_render` additionally ships with these DLLs:

 - libiconv https://www.gnu.org/software/libiconv/ (LGPL)
 - libfontconfig http://www.freedesktop.org/wiki/Software/fontconfig/
 - libxml2 http://xmlsoft.org/ (MIT)
 - liblzma http://tukaani.org/xz/ (Public Domain)
 
This font is included with both binary and source distributions:

 - Source Sans Pro - Regular https://github.com/adobe-fonts/source-sans-pro (SIL Open Font license)
 
Both binary and source builds are derived from Baseflight https://github.com/multiwii/baseflight (GPLv3) 
