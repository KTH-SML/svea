from threading import Thread, Event
from typing import Callable

import rospy

from svea.states import VehicleState
from svea_msgs.msg import VehicleState as VehicleStateMsg

__license__ = "MIT"
__maintainer__ = "Tobias Bolin, Frank Jiang"
__email__ = "tbolin@kth.se "
__status__ = "Development"

__all__ = [
    'LocalizationInterface',
]

class LocalizationInterface:
    """Interface handling the reception of state information from the
    localization stack.

    This object can take on several callback functions and execute them as soon
    as state information is available.

    Args:
        vehicle_name: Name of vehicle being controlled; The name will be
            effectively be added as a namespace to the topics used by the
            corresponding localization node i.e `namespace/vehicle_name/state`.
    """

    def __init__(
        self,
        vehicle_name: str = '',
    ):
        self.vehicle_name = vehicle_name
        sub_namespace = vehicle_name + '/' if vehicle_name else ''
        self._state_topic = sub_namespace + 'state'

        self.state = VehicleState()
        self.last_time = float('nan')

        self.is_ready = False
        self._ready_event = Event()
        rospy.on_shutdown(self._shutdown_callback)

        # list of functions to call whenever a new state comes in
        self.callbacks = []

    def start(self) -> 'LocalizationInterface':
        """Spins up ROS background thread; must be called to start receiving
        data.
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _wait_until_ready(self, timeout=20.0):
        tic = rospy.get_time()
        self._ready_event.wait(timeout)
        toc = rospy.get_time()
        wait = toc - tic
        return wait < timeout

    def _shutdown_callback(self):
        self._ready_event.set()


    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Localization Interface Node for "
                      + self.vehicle_name)
        self.node_name = 'localization_node'
        self._start_listen()
        self.is_ready = self._wait_until_ready()
        if not self.is_ready:
            rospy.logwarn("Localization not responding during start of "
                          "Localization Interface. Setting ready anyway.")
        self.is_ready = True
        rospy.loginfo("{} Localization Interface successfully initialized"
                      .format(self.vehicle_name))

        rospy.spin()

    def _start_listen(self):
        rospy.Subscriber(self._state_topic,
                         VehicleStateMsg,
                         self._read_state_msg,
                         tcp_nodelay=True,
                         queue_size=1)

    def _read_state_msg(self, msg):
        self.state.state_msg = msg
        self.last_time = rospy.get_time()
        self._ready_event.set()
        self._ready_event.clear()

        for cb in self.callbacks:
            cb(self.state)

    def add_callback(self, cb: Callable[[VehicleState], None]):
        """Add state callback.

        Every function passed into this method will be called whenever new
        state information comes in from the localization stack.

        Args:
            cb: A callback function intended for responding to the reception of
                state info.
        """
        self.callbacks.append(cb)

    def remove_callback(self, cb: Callable[[VehicleState], None]):
        """Remove callback so it will no longer be called when state
        information is received.

        Args:
            cb: A callback function that should be no longer used in response
            to the reception of state info.
        """
        while cb in self.callbacks:
            self.callbacks.pop(self.callbacks.index(cb))
