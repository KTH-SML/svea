#! /bin/sh
echo 'KERNEL=="i2c-[18]*", GROUP="i2c"' >> /etc/udev/rules.d/10-local_i2c_group.rules
echo 'KERNEL=="ttyACM[0-9]*", MODE="0666"' >> /etc/udev/rules.d/10-local_ttyACM.rules
adduser $SUDO_USER dialout
adduser $SUDO_USER i2c
udevadm control --reload-rules && \ 
    udevadm trigger --name=i2c-1 && \
    udevadm trigger --name=i2c-8 && \
    udevadm trigger --name=ttyACM0 && \
    udevadm trigger --name=ttyACM1

