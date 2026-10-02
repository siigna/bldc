# confsig

Computes `MCCONF_SIGNATURE` and `APPCONF_SIGNATURE` from a VESC Tool parameter
XML, so a signature can be derived and checked without building VESC Tool.

    ./confsig.py ../../../vesc_tool/res/config/7.01/parameters_appconf.xml

The firmware rejects a configuration whose signature does not match the one
compiled into `confgenerator.h`, and it does so silently: `COMM_SET_APPCONF`
prints a warning to the terminal and sends no reply, so the tool sees a timeout
rather than an error. Being able to check the signature independently turns that
into something visible before flashing.

This reimplements `ConfigParams::getSignature()`: the parameter name, its `type`
as a decimal, its `vTx` as a decimal, and each of its enum names are
concatenated in serialisation order, then CRC32-C is taken over the UTF-8 bytes.
Note that a parameter with no `<vTx>` element contributes `0`, the
`VESC_TX_UNDEFINED` the `ConfigParam` constructor leaves in place — 45 of the
appconf parameters are in that position, so it is not an edge case.

Verified against the values checked in for stock 7.01:

| config | signature |
|---|---|
| appconf | 296593100 |
| mcconf | 3154770096 |

## checkconf.py

Checks that `confgenerator.c` and `confgenerator.h` agree with the XML they were
generated from, in both the signature and the field order:

    ./checkconf.py ../.. ../../../vesc_tool/res/config/7.01

Worth having because the signature cannot catch an error in `confgenerator.c`
alone: it is a constant stored in the header, not something derived from the
serialisation code. Swap two adjacent fields in `confgenerator.c` and the
signature still matches, while every field from that point on is written to the
wrong place and the configuration still loads.
