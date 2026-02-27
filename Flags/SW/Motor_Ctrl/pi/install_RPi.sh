#!/bin/bash

# pygame: installed by default
# sudo apt -y install python3-pygame

# smbus is I2C library.
# smbus: installed by default
# sudo apt -y install python3-smbus

# Enable I2C1
sudo raspi-config nonint do_i2c 0
# Add comment
sudo sed -i '/dtparam=i2c_arm=on/s/$/ #I2C1/' /boot/firmware/config.txt
# Enable I2C0
sudo sed -i '/dtparam=i2c_arm=on/a dtparam=i2c_vc=on #I2C0' /boot/firmware/config.txt

echo "Please reboot RPi"
echo "sudo reboot"