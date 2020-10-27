echo 'KERNEL=="i2c-[18]*", GROUP="i2c"' >> /etc/udev/rules.d/10-local_i2c_group.rules
adduser $SUDO_USER dialout
adduser $SUDO_USER i2c
udevadm control --reload-rules && udevadm trigger --name=i2c-1 && udevadm trigger --name=i2c-8 

