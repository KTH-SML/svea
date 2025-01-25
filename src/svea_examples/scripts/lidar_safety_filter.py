# Authors: Magdalena Björk and Carl Elgcrona
# January 2025

import rospy
import numpy as np
from svea.sensors import Lidar

from geometry_msgs.msg import Twist

class LidarSafetyFilter:
    """
    This class manages the safety of Svea when remote driven and has 
    functions for speed reduction, turning aid and emergency stop.

    Parameters:
    :param IS_SIM: Toggles simulation
    :type IS_SIM: bool
    :param svea: SVEA container
    :type svea: class
    :param state: SVEA's state (unused parameter)
    :param task: Choice of task, if remote control is to be toggled on and off. 
                 Default value: "remote_control"
    :type task: str

    Subscribers:
    foxglove_teleop_callback: Takes input steering from the Teleop panel in foxglove
    webapp_steering_callback: Takes input steering from the webapp
    """

    #from svea.Team2.SVEA_Callbacks import park_button_callback, foxglove_remotedrive_callback, webapp_remotedrive_callback  
    

    def __init__(self, IS_SIM, svea, state=None, task = "remote_control"):

        self.IS_SIM = IS_SIM 
        self.svea = svea
        self.TASK = task

        self.remotedrive_v = 0
        self.remotedrive_yaw = 0
        self.lidar_steering = 0.0
        self.previous_steering = 0.0
        self.EMERGENCY_STOP = False
        self.angles = None

        self.MAX_SPEED = 1.0
        
        self.lidar = Lidar()
        self.lidar.start()
        self.lidar.add_callback(self.lidar_callback)

        rospy.Subscriber('/teleop_steering',Twist,self.foxglove_teleop_callback)
        rospy.Subscriber('/fmq/steering', Twist, self.webapp_steering_callback)


    def calculate_proportional_steering(self, data, data_right_sector, data_left_sector):
        """
        This function calculates the proportional steering to send to Svea. The steering
        uses a P-controller for each Lidar beam, weighted by its angle and distance, 
        and is added to the stering inputed by the user. 
        """
        # Initialize cumulative steering
        cumulative_steering = 0.0
        stop = False
        sim_weight = 1 if self.IS_SIM else 60/481 # Relation between number of rays simulation/real life

        # Constants
        MIN_DIST     = 0.05  
        MAX_DIST     = 0.75   
        MIN_STEERING = -1.7    
        MAX_STEERING = 1.7
        alpha = 0.5  # Smoothing factor between 0 (no smoothing) and 1 (full smoothing)

        # Prepare scan array
        data = np.array(data)
        data = np.where(np.isnan(data), 9999, data)
        if data.size == 0:
            rospy.logwarn("No valid LiDAR data available for steering calculation.")
            self.lidar_steering = 0
            return stop

        # If an object is within the right/left sector of the Lidar, prohobit turning in that direction
        if not self.EMERGENCY_STOP:
            if min(data_right_sector) < 0.35:
                MAX_STEERING = -self.remotedrive_yaw
            elif min(data_left_sector) < 0.35:
                MIN_STEERING = -self.remotedrive_yaw
                
        for i, distance in enumerate(data):
            angle = self.angles[i]
            if MIN_DIST < distance <= MAX_DIST:
                distance_weight = (1 - (distance - MIN_DIST) / (MAX_DIST - MIN_DIST))*sim_weight 
                angle_weight    = np.cos(1.5*angle)**3  
    
                # Steer away from obstacle: positive error if obstacle on right, negative if obstacle on left
                error = (-np.sin(angle)) * distance_weight * angle_weight

                # Accumulate error
                cumulative_steering += error 

        cumulative_steering = np.clip(cumulative_steering, MIN_STEERING, MAX_STEERING)

        # Apply low-pass filter (exponential moving average) to smooth steering
        self.lidar_steering = alpha * cumulative_steering + (1 - alpha) * self.previous_steering

        # Store the current steering for the next iteration
        self.previous_steering = self.lidar_steering

        return stop
    

    def lidar_callback(self, scan, angle_min, angle_increment): 
        """
        Function handling the speed reduction, turning aid and emergency stop. 
        The function is called every time a new Lidar scan is received.
        """
        # Prepare scan array
        scan = np.array(scan)
        scan = scan[np.isfinite(scan)]
        if scan.size == 0:
            rospy.logwarn("No valid LiDAR data available for emergency stop check.")
            return

        # Slice out different sectors of the Lidar, used for different safety features. 
        # Simulation and real life uses different amounts of rays
        if self.IS_SIM:                                                                 # 132 scans
            # Using the simulation Lidar
            data = scan[36:len(scan)-36]                                                # 60 scans/122 deg
            data_left_sector = scan[0:36]
            data_right_sector = scan[len(scan)-36:len(scan)]
            ms_front = np.min(data[int(len(data)/2)-2:int(len(data)/2)+2])              # 4 scans/8 deg
            ms_emergency_stop = np.min(scan[int(len(scan)/2)-22:int(len(scan)/2)+22])   # 44 scans/90 deg

        else:                                                                           # 1081 scans
            # Using the real Lidar
            data = scan[300:len(scan)-300]                                              # 481 scans/120 deg
            data_left_sector = scan[30:300]
            data_right_sector = scan[len(scan)-300:len(scan)-30]
            ms_front = np.min(data[int(len(data)/2)-16:int(len(data)/2)+16])            # 32 scans/8 deg
            ms_emergency_stop = np.min(scan[int(len(scan)/2)-360:int(len(scan)/2)+361]) # 361 scans/90 deg
        
        if self.angles is None or len(data) > len(self.angles):
            self.angles = [-np.pi/3 + i*angle_increment for i in range(len(data)+2)]

        ms = np.min(data)
        
        # Emergency stop if an obstacle is too close
        if self.EMERGENCY_STOP is False and ms_emergency_stop < 0.15:
            rospy.loginfo(f"EMERGENCY STOP: lidar-scan < 0.15")
            self.remotedrive_v = 0
            self.EMERGENCY_STOP = True
        elif self.EMERGENCY_STOP and ms_emergency_stop > 0.3:
            rospy.loginfo(f"No longer in emergency stop, continue driving.")
            self.remotedrive_yaw = 0
            self.EMERGENCY_STOP = False

        # Limit the maximum speed if an obstacle is close
        elif ms < 1.2:
            self.MAX_SPEED = max(np.arctan(6*(ms-0.15))*(2/np.pi)+0.15, 0.15) 
        else:
            self.MAX_SPEED = 1.0

        if ms_front < 3:
            self.MAX_SPEED *= (ms_front-1)**2/8+0.5

        # Pass the scan data and other parameters to compute the proportional steering
        self.calculate_proportional_steering(data, data_right_sector, data_left_sector)


    def foxglove_teleop_callback(self,msg):
        """
        Callback function for Teleop panel
        """
        x = msg.linear.x
        y = msg.linear.y

        self.remotedrive_v +=x
        self.remotedrive_yaw +=y


    def webapp_steering_callback(self,msg):
        """
        Callback function for webapp (same as in fmq_control.py)
        """
        self.remotedrive_yaw = -np.pi/2 * msg.angular.z
        self.remotedrive_v = 1.0 * msg.linear.x


    def remote_spin(self):
        """
        Spin function for safe remote control
        """

        if self.EMERGENCY_STOP:
            # Reverse if in emergency stop mode
            v = -0.15
            yaw = 0
        else: 
            v = min(self.remotedrive_v, self.MAX_SPEED)
            yaw = np.clip(self.remotedrive_yaw + self.lidar_steering, -1, 1)

        self.svea.send_control(yaw,v)
    

    def stop(self):
        """
        Function to stop the safe remote control
        """
        rospy.loginfo("Stopping remote control")
        rospy.Subscriber('/teleop_steering',Twist,self.foxglove_teleop_callback).unregister()
        rospy.Subscriber('/fmq/steering', Twist, self.webapp_steering_callback).unregister()
        self.lidar.remove_callback(self.lidar_callback)


    def run(self):
        """
        Function to start the safe remote control
        """
        while self.TASK == "remote_control":
            self.remote_spin()
        else:   
            self.stop()
