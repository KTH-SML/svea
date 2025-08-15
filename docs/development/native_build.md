# Building the SVEA Stack Natively

## System Requirements
This library is developed on and intended for systems running:

1. Ubuntu 24.04 (installation tutorial [here](https://ubuntu.com/tutorials/tutorial-install-ubuntu-desktop#1-overview))
2. ROS2 Jazzy (installation instructions [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html))

If you do not want to install Ubuntu onto your computer, consider installing a
[virtual machine](https://www.osboxes.org/ubuntu/) or use
[docker](https://docs.docker.com/install/) with Ubuntu 24.04 images.

Install python dependencies by calling:

```
pip install --break-system-packages -r requirements.txt
```

The installation instructions later on will use `colcon build` instead of
`ament_make`, so you should also [install colcon using apt-get](https://colcon.readthedocs.io/en/released/user/installation.html).

If you had a pre-existing ROS2 Jazzy installation, please run:

```bash
sudo apt update
sudo apt upgrade
```

before continuing onto installing the library.

## Installing the library
Start by going to the folder where you want the code to reside.
For example, choose the home directory or a directory for keeping projects in.
Once you are in the chosen directory, use the command:

```bash
git clone https://github.com/KTH-SML/svea
```

to download the library. Then, a new directory will appear called
`./svea`. Go into the directory with command:

```bash
cd svea
```

To install all of the ROS dependencies that you are missing for this library run:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Finally, compile and link the libraries using:

```bash
colcon build --symlink-install
source install/setup.bash
```

To make sure the libraries are linked in the future, also call (**you need to replace
`<path-to-svea>` with the file path to wherever you cloned "svea", e.g.
`/home/nvidia/svea/install/setup.bash`**):

```bash
echo "source <path-to-svea>/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Note, you only need to do this once.**
