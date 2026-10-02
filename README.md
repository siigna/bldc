# ESCargot

Motor controller firmware. A fork of the VESC® firmware, with a Lua script
engine and first-class pedal-assist support added.

**Not affiliated with, endorsed by, or certified by Mr. Benjamin Vedder.**
VESC® is his registered trademark; see [TRADEMARKS.md](TRADEMARKS.md). This
firmware is compatible with VESC® Tool, which is what uploads scripts and
writes the configuration.

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

## What this fork adds

### A Lua script engine, as an alternative to LispBM

`USE_LUA=1` requires `USE_LISPBM=0`; the two engines are mutually exclusive
and nothing picks between them at runtime. On an STM32F405 that is not a style
choice — the LispBM build is 96% of the app partition, and dropping it is what
makes room.

```bash
make fw_75_100_V2 USE_LISPBM=0 USE_LUA=1
```

Measured on `75_100_V2`:

| | app flash | CCM (`.ram4`) |
|---|---|---|
| LispBM | 96.1% | 99.6% |
| Lua | 89.3% | 72.5% |

The same container format, the same `COMM_LISP_*` packets, so VESC® Tool's
existing upload, erase and REPL carry Lua unchanged. Bindings cover the motor,
the configuration (153 parameters), the CAN bus and the inputs.

146 of those come from a parameter table **generated** from the LispBM
extensions rather than retyped; the other seven are scaled or computed and are
written out by hand. `tests/conf_table` regenerates the table and fails if the
two drift, because a round-trip test cannot catch a parameter wired to the
field next to the right one — set-then-get agrees either way.

### First-class pedal assist

Sensor front-ends, closed-loop power control, walk assist, an eRider
GTL-T17-73 preset, and the PAS configuration reachable from `conf-set` and
`conf-get`. Tested across every board variant, with sanitizers, coverage and a
property fuzzer.

### A simulated STM32F405

`qemu-system-arm -M olimex-stm32-h405` models the part this firmware targets,
so the real ChibiOS kernel runs without a board — enough to exercise a thread,
a mutex, an event, a timed sleep and a working area in CCM. It found three
firmware bugs that reading the code had not, including that Lua's default
`LUAI_MAXCCALLS` of 200 needs 95 KB of stack against the 64 KB that exists.

What it cannot do is anything board-specific: no ADC, CAN, SPI or FOC timers,
no flash driver, and no meaningful wall-clock timing. See
[tests/qemu/README.md](tests/qemu/README.md), which is explicit about the
difference.

## Tests

One entry point, for everything that does not need hardware:

```bash
nix-shell -p gcc gnumake python3 gcc-arm-embedded qemu gtest cppcheck \
    clang-tools --run ./tests/check.sh
```

Ten plain-C suites, the PAS suites across every board variant, the parameter
table, seven QEMU images, address and undefined sanitizers, the configuration
signature, cppcheck, clang-tidy and coverage. A build failure is a failure
rather than a skip, and the exit status is what counts — a sanitizer abort and
a CPU fault both exit without printing the word FAIL.

Optional stages: `--fuzz` for a short fuzzing run, `--tree` for cppcheck over
the whole firmware.

## Upstream

`upstream` points at `vedderb/bldc` over HTTPS and is pull-only; its push URL
is deliberately set to `no-push`. Changes go to `origin`.

## Supported boards

All of them!

Check the supported boards by typing `make`

```
[Firmware]
     fw   - Build firmware for default target
                            supported boards are: 100_250 100_250_no_limits 100_500...
```

There are also many other options that can be changed in [conf_general.h](conf_general.h).

## Prerequisites

### On Ubuntu (Linux)/macOS
- Tools: `git`, `wget`, and `make`
- Additional Linux requirements: `libgl-dev` and `libxcb-xinerama0`
- Helpful Ubuntu commands:
```bash
sudo apt install git build-essential libgl-dev libxcb-xinerama0 wget git-gui
```
- Helpful macOS tools: 

```bash
brew install stlink
brew install openocd
```

### On Windows
- Chocolately: https://chocolatey.org/install
- Git: https://git-scm.com/download/win. Make sure to click any boxes to add Git to your Environment (aka PATH)

## Install Dev environment and build

### On Ubuntu (Linux)/MacOS
Open up a terminal
1.  `git clone https://github.com/siigna/bldc.git`
2.  `cd bldc`
3.  Continue with [On all platforms](#on-all-platforms)

### On Windows

1.  Open up a Windows Powershell terminal (Resist the urge to run Powershell as administrator, that will break things)
2.  Type `choco install make`
3.  `git clone https://github.com/siigna/bldc`
4.  `cd bldc`
5.  Continue with [On all platforms](#on-all-platforms)

### On all platforms

1.  `git checkout origin/master`
2.  `make arm_sdk_install`
3.  `make` <-- Pick out the name of your target device from the supported boards list. For instance, I have a Trampa **VESC 100/250**, so my target is `100_250`
4.   `make 100_250` <-- This will build the **VESC 100/250** firmware and place it into the `bldc/builds/100_250/` directory

## Building with Nix

Nix is a build tool which manages all dependencies. With [Nix flakes](https://nixos.wiki/wiki/Flakes)
enabled, build the general-purpose firmware package from the repository root.

```bash
nix build .#bldc-fw
```

The packaged firmware and `res_fw.qrc` are available under `result/`.
All boards are built by default. `make *_flash` helpers are not supported with nix.

## Other tools

**Linux Optional - Add udev rules to use the stlink v2 programmer without being root**
```bash
wget vedder.se/Temp/49-stlinkv2.rules
sudo mv 49-stlinkv2.rules /etc/udev/rules.d/
sudo udevadm trigger
```

## IDE
### Prerequisites
#### On macOS/Linux

- `python3`, and `pip`

#### On Windows
- Python 3: https://www.python.org/downloads/. Make sure to click the box to add Python3 to your Environment.

### All platforms

1.  `pip install aqtinstall`
2.  `make qt_install`
3.  Open Qt Creator IDE installed in `tools/Qt/Tools/QtCreator/bin/qtcreator`
4.  With Qt Creator, open the vesc firmware Qt Creator project, named vesc.pro. You will find it in `Project/Qt Creator/vesc.pro`
5.  The IDE is configured by default to build 100_250 firmware, this can be changed in the bottom of the left panel, there you will find all hardware variants supported by VESC

## Upload to a controller
### Method 1 - Flash it using an STLink SWD debugger

1.  Build and flash the [bootloader](https://github.com/vedderb/bldc-bootloader) first
2.  Then `_flash` to the target of your choice. So for instance, for the VESC 100/250: 
```bash
make 100_250_flash
```

### Method 2 - Upload firmware with VESC® Tool over USB

1.  Clone and build the firmware in **.bin** format as in the above Build instructions

In VESC® Tool

2.  Connect to the controller
3.  Navigate to the Firmware tab on the left side menu 
4.  Click on Custom file tab
5.  Click on the folder icon to select the built firmware in .bin format (e.g. `build/100_250/100_250.bin`)

##### [ Reminder : It is normal to see the controller disconnect during the firmware upload process ]  
#####  **[ Warning : DO NOT DISCONNECT POWER/USB to the controller during the upload process, or you will risk bricking your controller ]**  
#####  **[ Warning : ONLY DISCONNECT your controller 10s after the upload loading bar completed and "FW Upload DONE" ]**

6.  Press the upload firmware button (downward arrow) on the bottom right to start upload the selected firmware.
7.  Wait for **10s** after the loading bar completed (Warning: unplug sooner will risk bricking your controller)
8.  The controller will disconnect itself after new firmware is uploaded.

## In case you bricked your controller
you will need to upload a new working firmware to the controller.  
However, to upload a firmware to a bricked controller, you have to use a SWD Debugger.


## Contribute

Issues and pull requests here:
[github.com/siigna/bldc](https://github.com/siigna/bldc).

This fork has no forum or chat of its own. The upstream ones are not the place
for questions about it — they support the VESC® firmware, not this, and
sending traffic there would waste their time and misrepresent whose software
you are running.

If the change you want belongs upstream rather than here, upstream is the
better home for it: a fix to the motor control or a new hardware target
reaches far more people there than on this branch.

## Tags

Every firmware release has a tag. They are created as follows:

```bash
git tag -a [version] [commit] -m "ESCargot firmware version [version]"
git push --tags
```

## License

The software is released under the GNU General Public License version 3.0
