#! /usr/bin/env python3

import rospy

from svea.simulators.sim_SVEA import SimSVEA
from svea.states import VehicleState
from svea.svea_managers.svea_archetypes import SVEAManager
from svea.interfaces import LocalizationInterface
from svea.data import TrajDataHandler, RVIZPathHandler

from svea_msgs.msg import VehicleState as VehicleStateMsg

from svea.park.nonlinear_mpc import NonlinearMPC
from svea.park.dynamic_model import DynamicModel
from svea.park.speed_guard import SpeedGuard

from std_msgs.msg import Float64MultiArray

class VehicleManager(SVEAManager): 

    def __init__(self, current_state = "standby"):

        self.IS_SIM = load_param('~is_sim', False)
        self.RUN_LIDAR = load_param('~run_lidar', True)
        self.USE_RVIZ = load_param('~use_rviz', True)
        self.CTRL_TYPE = load_param('~ctrl_type', "PPC")
        self.state = VehicleState(*load_param('~state',(0, 0, 0, 0))) # x, y, yaw, speed

        self.speed_guard = SpeedGuard().start() # Keeps track of speed and stop if too high
        self.drive_controller = NonlinearMPC
        self.data_handler = RVIZPathHandler if self.USE_RVIZ else TrajDataHandler
        self.localizer = LocalizationInterface
        rospy.Subscriber("/state", VehicleStateMsg, self.vehicle_state_callback)

        # Flags for SM state transitions
        self.is_parked = False
        self.park_rest_time = 3
        self.is_on_road = True # TODO: Might want to make this depend on the localization node
        self.has_path = False
        self.path_finished = False
        self.path_request_sent = False
        self.obstacle = False
        
        self.current_state = current_state
        self.last_state = None
        self.path = None
        self.time_parked = 0
        
        self.POINTS = []
        
        rospy.Subscriber("/outdoor_localization_waypoint", Float64MultiArray, self.get_pts_callback)
        if self.IS_SIM:
            self.sim_model = DynamicModel(self.state) # Dynamic model for simulation
            self.simulator = SimSVEA(self.sim_model, # Start the simulator
                                     dt=0.01,   
                                     run_lidar=self.RUN_LIDAR,
                                     start_paused=False).start()

        SVEAManager.__init__(self, self.localizer, self.drive_controller, data_handler=self.data_handler)

        # Initiate svea manager control interface
        self.start(wait=True) # Inherited method from SVEAManager

    def is_resting(self):
        wait = rospy.get_time() - self.time_parked < self.park_rest_time
        if wait:
            rospy.loginfo_throttle(5, "Waiting in parking spot for {} seconds".format(self.park_rest_time))
        return wait


    def get_pts_callback(self, msg):
        temp = msg.data
        self.POINTS=[]
        for count, item in enumerate(temp):
            if count%2 == 0:
                self.POINTS.append([item, temp[count+1]])

    def vehicle_state_callback(self, msg):
        self.state.x = msg.x
        self.state.y = msg.y
        self.state.yaw = msg.yaw
        self.state.v = msg.v


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)