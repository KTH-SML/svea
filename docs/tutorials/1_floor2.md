# Floor 2 and Pure-Pursuit

In this tutorial, we will start interacting with the code base for the SVEA
platform using the new ROS 2 and rosonic framework. In particular, you will inspect the pure-pursuit 
examples we have implemented and improve upon them to get you ready for running the 
examples on the real vehicles.

In this tutorial, there are some criterion you need to fulfill, such as not running
into walls. However, the quality of your solution is up for you to
judge. Ultimately, this tutorial is designed for you to start getting your hands
dirty with your own automated driving system.

If you haven't already, please checkout the [installation instructions](https://github.com/KTH-SML/svea/blob/main/README.md#installation).
Then, enter into a Docker container by calling with the `util/run-dev` script.

## Prerequisites

Before starting, ensure you have:
1. ROS 2 Jazzy installed and workspace sourced
2. The svea_core workspace built: `colcon build --symlink-install && source install/setup.bash`
3. A basic understanding of Python and ROS 2 concepts

## Running the Floor 2 Example

We will use the Floor 2 example, so please make sure it works before proceeding
with this tutorial. Check this by running the following without any errors:

```bash
ros2 launch svea_examples floor2.xml is_sim:=true
```

Before you get started with implementing improvements to the Floor 2 example, you should first create your own version of the example that you can edit. You can do this by copying `svea_examples/launch/floor2.launch` into `<your project package>/launch/floor2.launch` and `svea_examples/scripts/pure_pursuit.py` into `<your project package>/scripts/`.

## Understanding the Pure Pursuit Implementation

The pure pursuit controller is now implemented using the rosonic framework. Let's examine
the key components:

### 1. Pure Pursuit Controller Class

The controller is located in `svea_core/src/svea_core/controllers/pure_pursuit.py`:

```python
class PurePursuitController:
    """
    Pure Pursuit controller for following a path.
    
    This controller computes steering and velocity commands based on the current
    state of the vehicle and a predefined trajectory.
    """
    
    k = 0.1          # look forward gain
    Lfc = 0.6        # look-ahead distance  
    K_p = 0.15       # speed control proportional gain
    K_i = 0.01       # speed control integral gain
    L = 0.324        # [m] wheel base of vehicle
    
    def compute_control(self, state, target=None):
        steering = self.compute_steering(state, target)
        velocity = self.compute_velocity(state)
        return steering, velocity
```

## Task 1: Addressing Overshoot

When you run the Floor 2 example as is, you will notice that at some turns the vehicle has a big overshoot and some times gets very close, or even intersects, with a wall. Since we will run this example on the real SVEA at then end of the tutorial, this overshoot is unacceptable, as the real SVEA might actually drive into a wall. Your first task will be to address this overshoot and avoid colliding with any walls.

Since this is the first issue you are looking into, instead of starting with the implementation right away, you should start by reading into how the software driving this example is set up (pun intended). The best place to start is to inspect the PurePursuitController class definition that is imported in pure_pursuit.py. You can find the pure-pursuit controller class definition in:

`src/svea_core/svea_core/controllers/pure_pursuit.py`

By looking at and understanding how the pure pursuit controller works, try to come up with an approach for reducing the overshoot. To address this problem you may want to read up on how pure pursuit controllers work.

## Task 2: Adding Emergency Stops

Now that you have addressed the overshooting problem, let's try adding an emergency stop capability to your implementation. Currently, the pure pursuit example just continuously tracks a path with no intention to stop. In the case where we do have an obstacle that we know about, we should have the ability to stop the SVEA automatically.

The most basic version of emergency stops that you should try in this task is stopping the SVEA if it comes too close to a list of obstacle points. Imagine that you already know where the obstacles are and they are represented by a list of XY coordinates (e.g. `[(x1, y1), (x2, y2), ...]`). Then, you should try to update your implementation to stop when the SVEA gets too close to these points. We will leave the definition of being "too close" up to you.

We recommend that you try to visualize these obstacle points to make it easier for you to develop this functionality and to make it easier to show to others. To make your life easier, there's already a number of visualization functions in` src/svea_core/svea_core/utils/markers.py`.

If this is "too easy" for you, then consider the following enhancements:

1. Use the Foxglove "Publish Point" button to create obstacles through the /clicked_point ROS topic
2.  Add the walls to the obstacle points by using the map (see svea_core/scripts/plot_map.py to see how to work with the map)


## Task 3: Park the Vehicle

Currently, the vehicle will continuously goes around the paths until you halt
the launch or if the vehicle encounters an obstacle (if you completed the previous task).
In this taks, we'd like you to give the vehicle the ability to stop at a desginated
location, what we will refer to as a "parking spot".

We recommend you start by adding a point in the trajectory where, if the vehicle
is close enough, or if no more points come after it, the vehicle will switch
to being finished. In other words, when `self.controller.is_finished` switches to being `true`.
Develop an approach that allows the vehicle to park at a good angle.

Similar to the previous task, if this is "too easy" for you, consider the following
enhancements:

1. Add an angle error tolerance and make sure the vehicle can park within that error tolerance
2. Combine the approach with the previous task and make sure the vehicle can park even when there are obstacles around the parking spot

## Testing the Solution

### Simulation Testing

1. **Test in simulation first**: Use the simulation environment to verify your improvements work correctly.

```bash
# Launch simulation
ros2 launch svea_core svea_launch.xml is_sim:=true

# Launch visualization
ros2 launch svea_core map_and_foxglove.xml map:=floor2

# Run your improved controller
ros2 run your_package improved_pure_pursuit_node
```

### Real Vehicle Testing

When ready to test on the real vehicle:

1. **Prepare the vehicle**: Follow the startup procedure in the SVEA operation tutorial
2. **Launch the hardware interface**:
```bash
# Launch svea
ros2 launch svea_core svea_launch.xml is_sim:=false

# Launch visualization
ros2 launch svea_core map_and_foxglove.xml map:=floor2
```
3. **Run your controller**:
```bash
ros2 run your_package improved_pure_pursuit_node
```

### Safety Checklist

Before running on real hardware:

- [ ] RC remote is on and ready for override
- [ ] Start with low target speeds (< 1 m/s)
- [ ] Test emergency stop functionality
- [ ] Verify localization is working correctly
- [ ] Check that safety margins are appropriate
- [ ] Have someone ready to hit the emergency stop

## Expected Outcomes

After completing this tutorial, you should have:

1. **Understanding of rosonic framework**: How to create declarative ROS 2 nodes
2. **Improved pure pursuit controller**: Better handling of turns and curves
3. **Emergency stop capability**: Obstacle detection and avoidance
4. **Parking functionality**: Precise positioning and orientation control
5. **Simulation to real workflow**: Experience with the complete development cycle

## Next Steps

In the next tutorial, we'll explore the lidar simulation capabilities and
how to integrate sensor data into your control algorithms.