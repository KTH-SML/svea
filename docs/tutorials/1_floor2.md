# Floor2 pure-pursuit tutorial

In this tutorial, we will start interacting with the code base for the SVEA
platform. In particular, you will inspect the pure-pursuit examples we have
implemented and improve upon them to get you ready for running the examples on
the real vehicles.

In this tutorial, there are some criterion you need to fulfill, such as planning
trajectories that will not allow the vehicle to collide into walls, not going
above a speed limit, etc. However, the quality of your solution is up for you to
judge. Ultimately, this tutorial is designed for you to start getting your hands
dirty with your own automated driving system.

We will use the single vehicle pure-pursuit example and the Floor2 example, so
please make sure they work before proceeding with this tutorial. Check this by
running the following without any errors:

```bash
roslaunch svea_core pure_pursuit.launch
```

and

```bash
roslaunch svea_core floor2.launch
```

## Improving the pure-pursuit implementations

There are details in the pure-pursuit examples that we will improve upon in this
tutorial. Specifically, we will work on the following issues:

1. there is an offset between the vehicle's velocity and the target velocity
2. the pure-pursuit controller does not stop at the end of the trajectory
3. the vehicle does not complete the track in the Floor2 example

For all of these issues, there are `#TODO`'s commented wherever you should start
looking when solving these three issues.

### Speed control

To work on speed control, it's recommended you use `pure_pursuit.launch` to test
your solution, as the created plot includes the simulated vehicle's real-time
velocity in the title of the plot. Try running it and see how the steady-state
velocity compares with the `target_velocity` that is set in:

`svea_core/scripts/core_examples/pure_pursuit.py'`.

You might find it strange that the velocity is offset and a little noisy
in simulation. This is an intentional feature of the simulation. While we cannot
hope to identically replicate the disturbances and model mismatch that happens
on the real vehicle, we have recreated some basic phenomenon that affect the
real vehicles. Most importantly, we add in some noise and disturbances into the
steering and velocity, so that closed-loop control systems are already required
in simulation. In the pure-pursuit example, we have already closed the loop on
steering, but it is your task to close the loop on velocity.

Since this is the first issue you are looking into, instead of implementing a
speed controller right away, you should start by reading into how the software
driving this example is set up (pun intended). The best place to start is to
inspect the `SVEAPurePursuit` class definition that is imported in
`core_examples/pure_pursuit.py`; do not forget to also look at what class
variables and methods `SVEAPurePursuit` inherits from `SVEAManager`.

Did you see how a controller object is passed in and used by a SVEA manager
object? Now that you have looked a bit into how the pure-pursuit SVEA manager is
set up, you should be ready to start looking at the controller definition
itself. You can find the pure-pursuit controller class definition in:

`svea_core/src/svea/controllers/pure_pursuit.py`

Now take a look at the `compute_velocity()` method. We can immediately see that
the method is performing open-loop control on the velocity of the vehicle and
just sends the `self.target_velocity` off to the actuation interface without
checking on the actual velocity of the SVEA vehicle. We recommend you implement
and tune a PID controller here. We have added placeholder class variables at the
top of the `PurePursuitController` definition for you to use for your
implementation. We leave up to your own judgement on whether you use all three
terms or a subset of them for your speed controller. However, here are some
ideas you should consider:
1. Saturating the velocity at a set speed limit
2. Anti-windup for situations where the vehicle has driven into something or if
   the electronic speed controller (ESC) is turned off.

and here are some potential ideas you could think about for the future:
1. How could you implement acceleration-based control when the actuation
   interface only accepts velocity?
2. How do you both prevent overshoot over your speed limit while ensuring the
   velocity is smooth around the speed limit?

Once you have implemented your controller, test it out by launching
`pure_pursuit.launch` and verifying that the velocity matches the velocity you
set. In `core_examples/pure_pursuit.py`, try adding a second target velocity and
change the target velocity of the vehicle after 3 seconds and observe how your
controller does with this change.

### Terminating condition

Now, try (if you haven't already) letting the pure-pursuit example continue to
run after the vehicle has finished the trajectory. You will notice that the
vehicle will continue to drive around trying to reach the last point on the
trajectory with the target velocity you have set. Typically, we would like the
vehicle to stop moving once it has reached the end of its trajectory.

In the `PurePursuitController` class definition, notice that the class has a
variable called `is_finished`. You probably already saw that this variable is
already used in the `compute_velocity()` method as a flag for setting the
velocity of the vehicle to zero. Thus, your task is to correctly switch this
variable from it's default value of `False` to `True` when the vehicle's
trajectory has been completed. We recommend you try doing this inside of the
internal method `_calc_target_index()` in:

`svea_core/src/svea/controllers/pure_pursuit.py`

Once you have tried setting the terminating condition correctly, try launching
`pure_pursuit.launch` to verify that the vehicle completes the trajectory in a
way that you are satisfied with or believe is correct. The "completion" of a
trajectory might be more subjective than you think.

### Completing a circuit in Floor2

With speed control and a trajectory-based terminating condition implemented, you
are ready to try the Floor2 example, which uses the `SVEAPurePursuit` manager
object that you have been working with. Try launching `floor2.launch` to
double-check that your updates to the `PurePursuitController` are still working
as you expect. You can check the velocity of the vehicle by either printing out
the state of the svea (we recommend you use `rospy.loginfo_throttle()`) or by
using `rostopic echo` in a new terminal.

After you have verified that your speed controller and the vehicle stops at the
end of the trajectory, you can try changing the trajectory in:

`svea_core/scripts/core_examples/floor2_example.py`

such that the vehicle completes a circuit around floor2 using the outer
hallways. The points in the trajectory are in the coordinate space of the map of
floor2. To find the points you want for your trajectory, you can use the
"Publish Point" tool in the toolbar of the RVIZ window. If you click on the
button to activate the tool, when you hover around the map, you will see the XY
coordinate of where your cursor is hovering in the bottom of the RVIZ window. If
you want to know the heading information of a specific point, use the "2D Nav
Goal" tool instead. If you open a new terminal and run `rostopic echo
/move_base_simple/goal`, then when you click using the nav goal tool, you will
see the data you want.

When choosing the points of the trajectory, try to make sure that the resultant
trajectory will not collide with any walls. Furthermore, after designing your
trajectory, try launching `floor2.launch` again and make sure that the polygon
of the vehicle doesn't intersect with a wall at any point.

The process of choosing a trajectory is fairly inconvenient, however, you should
not have to do it often. Ideally, you should be using a planner to automatically
generate planned trajectories for you in real-time. However, this
initial trajectory should serve as a good baseline that you can use to test
other parts of your automation pipeline.

## Testing the solution

Finally, you are ready to try out your initial solution on the real vehicles! To
do this, start out by carefully reading the "Going from simulation to real"
section in the README of this repository. Also, make sure you have read the
"Hardware" section in the introductory tutorial, so you understand how the
hardware and software are connected to each other.

Make sure you add the nodes and includes mentioned in the "Going from simulation
to real" section to the floor2 launch file. We recommend you add them with
`<group if>` tags surrounding them (see [3.1 in the ROS wiki roslaunch XML
page](http://wiki.ros.org/roslaunch/XML)), so that you can link them to whether
`is_sim` is true or not.

Then, try linking an external laptop to a real SVEA vehicle's ROS network, by
following the instructions in the "Listening to ROS on another computer" section
of the repo's README. Once you have established a connection, place the SVEA
vehicle in the area you want it to start in and **turn off the motor** by
switching off the ESC. You can try launching the launch file on the SVEA vehicle
while running an RVIZ client on your laptop. To run the RVIZ client on your
laptop using the same configuration as the floor2 simulator, from
`svea_core/resources/rviz/` run:

```bash
rviz -d SVEA_floor2.rviz
```

Now, confirm that all the data is publishing to RVIZ as you expect it to. Once,
you have confirmed everything runs as you expect it to, then shutdown the
launch to try out the controller. If you try turning on the motor before
shutting down the launch, then, depending on your speed controller
implementation, the vehicle may dangerously jump due to windup (i.e. the speed
controller sees that the velocity is zero and tries to send a higher velocity
signal to the esc until it maxes the velocity out). After shutting down the
launch, turn the ESC on, make sure the SVEA vehicle is still in the position you
want it in. Make sure you are holding the corresponding RC remote and are ready
to block the signal from your implementation using the metal switch on top of
the remote by switching it to the forward position. Then, with the metal switch
on top of the remote in the middle position, try running your launch once more,
and see what happens!
