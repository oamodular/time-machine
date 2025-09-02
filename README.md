# OAM Time Machine for Eurorack

## Expanders

Time Machine has methods for addings expaders. Via I2C pins or by sending/receiving Eurorack level signals to our 8-pin expander dealie. To be clear, these are **not currently used** by the firmware.

We made a [passive expander](https://github.com/oamodular/passive-expander) for working with the 8-pins on the back of Time Machine. This will add the following types of signals for working with: 1x Gate in, 4 CV in (16-bit, bipolar 5V), 2 CV out (12-bit unipolar 5v).
