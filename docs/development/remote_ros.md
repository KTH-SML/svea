# Listening to ROS on another computer

Since you will not be able to drive the SVEA cars with a display plugged in, it
can be useful to link a computer that does have a display to the SVEA car's ROS
network. This will let you use [RVIZ](http://wiki.ros.org/rviz) and
[PlotJuggler](http://wiki.ros.org/plotjuggler) on the computer with a display
while accessing the data streams on the SVEA car. This amounts to telling the
computer with a display where the ROS master it should be listening to (in this
case, it should listen to the ROS master running on the SVEA car) is located on
the network. We do this with a the SVEA's hostname, e.g. `svea3`. On the SVEA
car and the computer with a display, run:

```bash
# on SVEA
. <svea_root>/util/remote_ros.sh

# on computer
. <svea_root>/util/remote_ros.sh <hostname>
```

You can test if this worked by launching something on the SVEA car in the same terminal where the export commands were run and then calling ```rostopic list``` on the computer with a display in the same terminal where the export commands were run. You should see the topics you expect to be on the SVEA car also available on the computer with a display. If this worked, you have some options for how you want to use it. You can either:
1. call this script everytime you want to link the SVEA car and the computer with a display togther (the script only links the terminal window you run it in),
2. add an [alias](https://mijingo.com/blog/creating-bash-aliases) to the end of the SVEA car and the computer's ```~/.bashrc``` to create a new bash command,
3. you can add the contents of ```remote_ros.sh``` directly to the end of your ```~/.bashrc```,

or some other preferred approach.
