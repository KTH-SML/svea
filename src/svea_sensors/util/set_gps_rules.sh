#!/bin/bash

function set_udev_rules() {

    rules_path=/etc/udev/rules.d/rtk-gps.rules

    rules_content='SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps"'

    echo -e $rules_content | sudo tee $rules_path
    sudo udevadm control --reload-rules && sudo udevadm trigger
    echo "udev rules set successfully!"
}

set_udev_rules