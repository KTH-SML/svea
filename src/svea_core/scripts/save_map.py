#!/usr/bin/env python3

import pickle
from pathlib import Path
from datetime import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import time


class SaveMap(Node):

    def __init__(self):
        super().__init__('save_map')

        # declare parameters
        self.declare_parameter('save_dir', str(Path.home()/'maps'))
        self.declare_parameter('file_name', '%Y%m%d-%H%m')
        self.declare_parameter('save_method', 'numpy')

        # load parameters
        self.save_dir = self.get_parameter('save_dir').value
        self.file_name = self.get_parameter('file_name').value
        self.save_method = self.get_parameter('save_method').value

        self.get_logger().info(f'My svae directory is {self.save_dir}', once=True)

        # make sure save directory exist
        self.save_dir = Path(self.save_dir)
        self.save_dir.mkdir(parents=True, exist_ok=True)

    def map_cb(self, msg: OccupancyGrid):
        """
        Callback for when message is received on the /map topic.
        Using save_method we can choose how to save the map. The save method
        is determined by the method defined in this class.
        """

        path = self.save_dir
        path /= datetime.now().strftime('{self.file_name}')

        save_fn = getattr(self, f'save_{self.save_method}', None)

        if save_fn is None:
            raise Exception('Invalid save method!')
        else:
            save_fn(msg, path)

    def save_numpy(self, msg: OccupancyGrid, path: Path):
        """
        Save map to numpy file.

        If save_method == 'numpy' this method will be invoked. It takes the
        map, converts it to a numpy array and then saves that array using
        numpy's save function.
        """

        height = msg.info.height
        width = msg.info.width
        data = msg.data

        with open(f'{path}.npy', 'wb') as f:
            arr = np.array(data).reshape(height, width)
            np.save(f, arr)

    def save_pickle(self, msg: OccupancyGrid, path: Path):
        """
        Save map by pickling the object.

        If save_method == 'pickle' this method will be invoked. It takes the
        map message object and pickles it (serialization of the python object).
        To extract the object you need to have the OccupancyGrid type available
        during de-serialization.
        """

        with open(f'{path}.pickle', 'wb') as f:
            pickle.dump(msg, f)

    def save_bytes(self, msg: OccupancyGrid, path: Path):
        """
        Save map using the message objects own serialization method.

        ROS messages has their own serialization/de-serialization method for
        sending/receiving payloads in the lower level ROS communication layers.
        We can use these methods and simply save to file.

        How to retrieve:
            >>> from nav_msgs.msg import OccupancyGrid
            >>> with open('<path>', 'rb') as f:
            ...     msg = OccupancyGrid().deserialize(f.read())
        """

        with open(path, 'wb') as f:
            msg.serialize(f)

def wait_for_message(node ,topic_type, topic): 
    class _vfm(object):
        def __init__(self) -> None:
            self.msg = None
            
        def cb(self, msg):
            self.msg = msg

    vfm = _vfm()
    subscription = node.create_subscription(topic_type,topic,vfm.cb,1)
    while rclpy.ok():
        if vfm.msg != None: return vfm.msg
        rclpy.spin_once(node)
        time.sleep(0.001)
    # unsubcription
    subscription.destroy()

def main(args=None):
    rclpy.init(args=args)
    node = SaveMap()
    msg = wait_for_message(node, OccupancyGrid, "/map")
    node.map_cb(msg)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()