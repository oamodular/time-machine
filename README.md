# OAM Time Machine for Eurorack

## Install and Build

Make sure to clone this repo recursively:

```
git clone --recurse-submodules https://github.com/oamodular/time-machine
```

You will also need some arm eabi packages, which you can get on a debian-ish system with:

```
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
```

Now finally to build:

```
(cd DaisyExamples/libDaisy && make) \
&& (cd DaisyExamples/DaisySP && make) \
&& (cd TimeMachine && make)
```

## Expanders

Time Machine has methods for addings expaders. Via I2C pins or by sending/receiving Eurorack level signals to our 8-pin expander dealie. To be clear, these are **not currently used** by the firmware.

We made a [passive expander](https://github.com/oamodular/passive-expander) for working with the 8-pins on the back of Time Machine. This will add the following types of signals for working with: 1x Gate in, 4 CV in (16-bit, bipolar 5V), 2 CV out (12-bit unipolar 5v).
