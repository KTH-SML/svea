# Floor 2 and Pure-Pursuit

In this tutorial, we will start interacting with the code base for the SVEA
platform. In particular, you will inspect the pure-pursuit examples we have
implemented and improve upon them to get you ready for running the examples on
the real vehicles.

In this tutorial, there are some criterion you need to fulfill, such as not running
into walls. However, the quality of your solution is up for you to
judge. Ultimately, this tutorial is designed for you to start getting your hands
dirty with your own automated driving system.

If you haven't already, please checkout the [installation instructions](../../README.md#installation).
Then, enter into a Docker container by calling with the `util/run-dev` script.

We will use the Floor 2 example, so please make sure they work before proceeding
with this tutorial. Check this by running the following without any errors:

```bash
roslaunch svea_examples floor2.launch
```

This launch file uses the python script called `pure_pursuit.py` which can be
can be found in `svea_examples/scripts`. This python script will be
your starting point for this tutorial.

Before you get started with implementing improvements to the Floor 2 example,
you should first create your own version of the example that you can edit. You
can do this by copying `svea_examples/launch/floor2.launch` into `<your
project package>/launch/floor2.launch` and `svea_examples/scripts/pure_pursuit.py` into
`<your project package>/scripts/`.

## Task 1: Addressing overshoot

When you run the Floor 2 example as is, you will notice that at some turns
the vehicle has a big overshoot and some times gets very close, or even
intersects, with a wall. Since we will run this example on the real SVEA at
then end of the tutorial, this overshoot is unacceptable, as the real SVEA
might actually drive into a wall. Your first task will be to address this
overshoot and avoid colliding with any walls.

Since this is the first issue you are looking into, instead of starting with
the implementation right away, you should start by reading into how the software
driving this example is set up (pun intended). The best place to start is to
inspect the `SVEAPurePursuit` class definition that is imported in
`pure_pursuit.py`; do not forget to also look at what class
variables and methods `SVEAPurePursuit` inherits from `SVEAManager`.

Did you see how a controller object is passed in and used by a SVEA manager
object? Now that you have looked a bit into how the pure-pursuit SVEA manager is
set up, you should be ready to start looking at the controller definition
itself. You can find the pure-pursuit controller class definition in:

`svea_core/src/svea/controllers/pure_pursuit.py`

By looking at and understanding how the pure pursuit controller works, try to
come up with an approach for reducing the overshoot. To address this problem
you may want to read up on how pure pursuit controllers work.

## Task 2: Adding emergency stops

Now that you have addressed the overshooting problem, let's try adding an
emergency stop capability to your implementation. Currently, the pure pursuit
example just continuously tracks a path with no intention to stop. In the
case where we do have an obstacle that we know about, we should have the
ability to stop the SVEA automatically.

The most basic version of emergency stops that you should try in this task
is stopping the SVEA if it comes too close to a list of obstacle points. Imagine
that you already know where the obstacles are and they are represented by a
list of XY coordinates (e.g. `[(x1, y1), (x2, y2), ...]`). Then, you should
try to update your implementation to stop when the SVEA gets too close to
these points. We will leave the definition of being "too close" up to you.

We recommend that you try to visualize these obstacle points to make it
easier for you to develop this functionality and to make it easier to show
to others. To make your life easier, there's already a number of visualization
functions in `svea_core/src/svea/simulators/viz_utils.py`.

If this is "too easy" for you, then consider the following enhancements:

1. Use the Foxglove "Publish Point" button to create obstacles through the `/clicked_point` ROS topic
2. Add the walls to the obstacle points by using the map (see `svea_core/scripts/plot_map.py` to see how to work with the map)

## Task 3: Park the vehicle

Currently, the vehicle will continuously goes around the paths until you halt
the launch or if the vehicle encounters an obstacle (if you completed the previous task).
In this taks, we'd like you to give the vehicle the ability to stop at a desginated
location, what we will refer to as a "parking spot".

We recommend you start by adding a point in the trajectory where, if the vehicle
is close enough, or if no more points come after it, the vehicle will switch
to being finished. In other words, when `self.svea.is_finished` switches to being `true`.
Develop an approach that allows the vehicle to park at a good angle.

Similar to the previous task, if this is "too easy" for you, consider the following
enhancements:

1. Add an angle error tolerance and make sure the vehicle can park within that error tolerance
2. Combine the approach with the previous task and make sure the vehicle can park even when there are obstacles around the parking spot

## Testing the solution

Finally, you are ready to try out your initial solution on the real vehicles!
Also, make sure you have read the "Hardware" section in the introductory tutorial,
so you understand how the hardware and software are connected to each other.

Place the SVEA vehicle in the area you want it to start in and **turn off the motor** by
switching off the ESC. You can try launching the launch file on the SVEA vehicle
while running Foxglove Studio on your laptop.

Now, confirm that all the data is publishing to Foxglove Studio as you expect it to. Once,
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
