#!/bin/sh

# switch into bootloader
python3 ./examples/mainmode-switch.py

sleep .5

# flash over rs485 wires
./modbus-flash 0x01 ../main.hex

sleep .5

# switch back to main
python3 ./examples/bootmode-switch.py

