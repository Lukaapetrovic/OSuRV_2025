#!/bin/bash

#sudo /opt/nvidia/jetson-io/jetson-io.py

# VCC 1
# GND 6
# SDA 3
# SCL 5
i2cdetect -y -r 1
# VCC 17
# GND 25
# SDA 27
# SCL 28
i2cdetect -y -r 0

# All off
i2cset -f -y 1 0x27 0x00
# All on
i2cset -f -y 1 0x27 0xf0


i2cset -f -y 0 0x20 0x00
i2cset -f -y 0 0x27 0x00

i2cset -f -y 0 0x20 0xff
i2cset -f -y 0 0x27 0xff

i2cset -f -y 0 0x20 0x10
i2cset -f -y 0 0x27 0x32
