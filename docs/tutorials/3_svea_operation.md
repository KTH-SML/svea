<!--TODO: move some of the commented out instructions to a course tutorial-->

# The SVEA

This document contains instructions for working with the SVEA platform.

__If you have an issue, please check the Common Problems section at the bottom.__

First are the basic rules that should always be followed when working with the cars. The second section is a description of some of the components of the car followed by: startup and shut down procedures, instructions for how to interface with the car and finally some common problems and solutions.

## Basic tips for working with the cars

* Always keep the wheels of the table or on the floor when you turn on the ESC.
<!--* Do not modify the cars to enhance their performance without contacting Tobias first. Adjusting the suspension is the exception to this rule.-->
<!--* Do not "borrow" parts from other teams cars to replace ones that you suspect are broken on your own. Ask Frank or Tobias for support first.-->
* Someone should always be ready with the remote override when the car is driving autonomosly.
<!--* __Always unplug all charging batteries if you are the last person to leave the project room for the day!__-->


## SVEA components and terminology
We start by introducing the components of the SVEA that is relevant when working on the physical vehicle. Hopefully this will establish common names for the components and give you a better understanding of how the SVEA works. The sensors are already described in the intro tutorial so they will not be dealt with here.

### TRX-4
The chassis is a Traxxas TRX-4 RC car. It is a 4 wheel drive electrically driven car. The steering is done by the front wheels (as you would expect) and is actuated by a steering servo. The car has two gears, controlled by another servo. It can also lock and unlock the front and rear differentials, controlled by one servo each.

For more information on the TRX-4, see the [TRX-4 owners manual](https://traxxas.com/sites/default/files/82056-4-OM-EN-R01.pdf)

### Battery box
The Battery box is mounted on top of the chassis and contains the batteries that the SVEA needs. It also serves as a mounting point for various other components.

### Motor Battery
The motor battery is a standard NiMh RC battery. It is located in the small, left hand side compartment of the battery box. It provides power to the main motor and the servos.

### Electronic Speed Control (ESC)
The ESC is located underneath the battery box, to the left. It is connected to the motor battery and controls the current to the main motor.

For more information on the ESC, see the [TRX-4 owners manual](https://traxxas.com/sites/default/files/82056-4-OM-EN-R01.pdf) page 18-20.

### Power bank
The power bank is a large LiPo power bank. It is located in the large right hand side compartment of the battery box.
The power bank has a power on button (it will power off automatically when it does not detect a sufficient power draw) and a small LCD screen indicating the current charge percentage.  The power bank provides power to everything on the SVEA that the motor battery does not provide power for. It mainly powers the TX2, the USB hub, and various sensors.

### TX2
The Nvidia Jetson TX2 is the main computer of the SVEA. It is located on top of the battery box. **The power on button is located on the left side.** It is the closest to the front, out of 4 buttons. On the left side, there is a USB port, a HDMI port, an ethernet port, and the WiFi antennas.

When power is connected to the TX2 a small red LED on the TX2 should turn on. When the TX2 is powered on two additional green LEDs on the TX2 should also turn on.

### Remote
An RC remote that came with the TRX-4. As a normal RC remote it has a throttle and a steering wheel. In addition, it also has a three state switch on the top (which controls the differential locks and override mode) and a thumb switch to control the gear.

The remote also has an LED which is normally green when the remote is on. If the LED flashes with a red light, the batteries needs to be replaced.

For more information on the remote, see the [TRX-4 owners manual](https://traxxas.com/sites/default/files/82056-4-OM-EN-R01.pdf)
The various LED signals are described on page 32.

### Signal and power managment board (SPMB) (AKA "power board")
The power board is located beneath the battery box, on the right hand side of the vehicle. It's a blue PCB with switches, 4 LEDs, 4 buttons, several power outlets, and a picture of a duck on it.
The power board has two main purposes: Regulate the power from the power bank and distribute it to the rest of the SVEA, and manage low level control signals to the ESC and the servos. The brain of the SPMB is a Teensy 4.0 micro controller, which is connected to the TX2 through USB.

The SPMB has two switches: one for toggling the 12V power supply and one for toggling the servo power. The switches have built in LEDs that will be green when on and red when off. The servo switch can also shine yelow if it is in the ON position but the Teensy is not sending a servo power signal. No servo power signal is normal if there is no signal comming from either the remote or the TX2.

The 4 LEDs are controlled by the teensy. They are labeled 0-3 starting from the front. In normal operation they will indicate the status of the low level interface (LLI) flags. Green means that the flag is set, and red means that it is not.
A green light means that:

0. The TX2 is sending signals to the SPMB
1. A signal from the receiver is detected by the SPMB
2. The remote is in override mode
3. The emergency brake is engaged

If none of the flags are set, the LEDs will display a rolling patterns of various colors (aka disco mode).

The receiver reset button are to the left of the LEDs. This button can be used to power cycle the receiver.

There are other buttons located just in front of the LEDs. __DO NOT TOUCH THESE BUTTONS!__

To the far right on the power board is the large connector that connects to the power bank.

### Receiver
The receiver receives control signals from the remote and passes them on to the power board. The reciver has one LED. If it is green, then the remote is connected, if it is red, then the receiver is waiting, and if it is flashing red, then the receiver has given up on waiting for the remote (make sure that the remote is on and try pressing the receiver reset button).

For more information on the receiver, see the [TRX-4 owners manual](https://traxxas.com/sites/default/files/82056-4-OM-EN-R01.pdf)

### USB Hub
The USB hub connects all USB devices to the TX2. It is mounted on the rear of the SVEA.

### 4G WiFi modem
The 4G modem provides independent internet acces to the SVEA.
The modem is mounted at the back, underneath the USB hub.
The TX2 receives the internet connection over USB.
Other computers can connect to the modem over WiFi.

The modem has two buttons, a big power button on the front and a smaler menu button on top.

## Procedures for turning the car ON and OFF

### Turn ON 12V power
1. Plug in the powerbank cord to the powerbank. (Alternatively, plug the 19V power adapter into the power board.)
2. Push the button on the power bank. The switches on the powerboard should now turn red.
3. Turn ON the 12V switch. It should turn green. The LEDs on the powerboard should start displaying the idle pattern and a red LED on the TX2 should turn on.

### Turn ON the TX2
These steps assume that 12V power is already ON.
1. Push the ON button on the TX2. Two green LEDs on the TX2 should turn on.

### Turn ON motor and servo power
These steps assumes that 12V power is already ON.
1. Make sure that the SVEA is either on the floor or propped up so that the wheels are not touching the table.
2. Make sure that the servo switch is in the OFF position (it should be red).
3. Turn ON the remote. The LED on the remote should turn green.
4. Make sure that the LED on the receiver shines with a steady green light. If it blinks red: push the receiver reset button on the power board.
5. Plug in the motor battery.
6. Press the ESC button until the LED lights up (but don't hold the button down after the LED lights up!).
7. Turn ON the servo switch. It should turn green.

### Turn OFF the TX2
1. Turn OFF the TX2 just as you would turn OFF a normal computer.

### Turn OFF motor and servo power
1. Turn OFF the sevo power switch on the power board.
2. Press the button on the ESC until the LED turns OFF.
3. Disconnect the motor battery from the ESC.
4. Turn OFF the remote.

### Turn OFF the 12V power
1. Make sure that the TX2 is turned OFF.
2. Flip the 12V switch on the powerboard to the OFF position. It should turn red.
3. Unplug the powerbank connector. No lights should remain after this.

### Before putting the SVEA into storage
1. __The powerbank should be disconnected__
2. __The motor battery should be disconnected__
3. __The remote should be OFF__

Always check these things before leaving a SVEA!

## Interfacing with the car
The simplest way to interface with the car is to use an HDMI cable to connect the TX2 to a screen.

### Connecting to WiFi and using SSH
When driving you can connect to the car over SSH.
Start by connect your computer to the WiFi of the 4G modem on the car.
If you are unsure if the 4G modem is turned on: take the modem out from the SVEA and press the big button. If nothing appears on the screen, try holding down the button for 5 seconds.

You can find the SSID and password by first taking the modem out of the car, and then pressing the small button on top until the WiFI information  is shown on the screen.
Once the 4G modem is on and your computer is connected to the 4G modem WiFi you should be able to connect to the svea by typing in __(replace # with the number of your car)__ `ssh nvidia@svea#.local`  in a terminal.

You should also see the _listening to ROS on another computer_ section in the [README](https://github.com/kth-sml/svea) for how to use the remote features of ROS.

## Common Problems

### Realsense T265 not found when launching localize.launch
Unplug and plug back in the RS T265 USB cable.
Due to a quirk with how the T265 starts, this will need to be done every time the TX2 is booted.

### ESC LED blinks with a fast green light
Normal if the servo switch is OFF or no signal is sent from the SPMB.
If a signal is being sent and the servos switch is green, the ESC has likely been calibrated incorrectly.
See TRX-4 owner's manual for how to calibrate the ESC.

### ESC LED is blinking with a slow red light or shines with a green light
The ESC is in LiPo (different type of battery than what we are using) mode instead of NiMH mode and the under voltage protection has been triggered. See the TRX-4 owner's manual for how to switch the ESC to NiMH mode.

### Remote LED is blinking with a slow red light
Remote batteries are low on power.
Replace the batteries in the remote.

### The 4G modem refuses to turn on
The rechargable battery of the 4G modem is likely out of power.
Try connecting the 4G modem to a USB port and push the button for 5 seconds.
A charging symbol should appear on the screen.
Let the modem charge for at least 15 minutes, unplug the USB cable and try pressing the button for 5 seconds again.
The battery in the 4G modem will usually run out in a couple of days if it is left on and does not receive power from another device.
