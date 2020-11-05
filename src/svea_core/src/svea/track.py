"""
Track handling module. Contains classes for handling tracks.
"""

from threading import Thread
import rospy
from geometry_msgs.msg import (PolygonStamped,
                               Point32,
                               PointStamped,
                               PoseWithCovarianceStamped,
                               PoseStamped)
try:
    from yaml import load, dump
except ImportError:
    pass


__license__ = "MIT"
__maintainer__ = "Tobias Bolin"
__email__ = "tbolin@kth.se"
__status__ = "Development"


class Track(object):
    """ Representation of a track.
    Useful for loading the track parameters.
    Can publish the track as polygons for visualization in rviz.

    The track is defined by one stay inside polygon and a keep out polygon.
    The coordinates are defined relative to a given occupation grid map.

    The track class is intended as a unified way of defining
    track boundaries in a way that is easy to relate to the real world.
    It is not intended to be ideal for path planning and similar tasks.

    :param vehicle_name: Name of vehicle
    :type vehicle_name: str, optional
    :param publish_track: If the track should be published
    for visualization in rviz. Default: False
    :type publish_track: bool, optional
    """

    def __init__(self, vehicle_name='', publish_track=False):

        self.vehicle_name = vehicle_name
        self.stay_in_topic = 'track/stay_in'
        self.keep_out_topic = 'track/keep_out'
        self.publish_track = publish_track
        self._stay_in = None
        self._keep_out = None

    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data

        :return: itself
        :rtype: Track
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Track Node: \n" + str(self))
        if self.publish_track:
            self._start_publish()
            self._publish_track()
        rospy.loginfo("{} Track successfully initialized".format(
            self.vehicle_name))
        rospy.spin()

    def _start_publish(self):
        self.stay_in_pub = rospy.Publisher(self.stay_in_topic,
                                           PolygonStamped,
                                           queue_size=1,
                                           latch=True)
        self.keep_out_pub = rospy.Publisher(self.keep_out_topic,
                                            PolygonStamped,
                                            queue_size=1,
                                            latch=True)

    def _publish_track(self):
        stamp = rospy.Time.now()
        stay_in_msg = PolygonStamped()
        stay_in_msg.header.stamp = stamp
        stay_in_msg.header.frame_id = 'map'
        stay_in_msg.polygon.points = list_to_polygon(self.stay_in)
        self.stay_in_pub.publish(stay_in_msg)

        keep_out_msg = PolygonStamped()
        keep_out_msg.header = stay_in_msg.header
        keep_out_msg.polygon.points = list_to_polygon(self.keep_out)
        self.keep_out_pub.publish(keep_out_msg)
        rospy.loginfo('Track published')

    def _get_stay_in_from_parameters(self):
        stay_in = rospy.search_param('stay_in')
        self._stay_in = rospy.get_param(stay_in)
        self._stay_in.append(self._stay_in[0])

    def _get_keep_out_from_parameters(self):
        keep_out = rospy.search_param('keep_out')
        self._keep_out = rospy.get_param(keep_out)
        self._keep_out.append(self._keep_out[0])

    @property
    def stay_in(self):
        """The stay inside polygon"""
        if self._stay_in is None:
            self._get_stay_in_from_parameters()
        return self._stay_in

    @property
    def keep_out(self):
        """The keep out polygon"""
        if self._keep_out is None:
            self._get_keep_out_from_parameters()
        return self._keep_out


def list_to_polygon(node_list, z=0.3):
    polygon = [Point32(*(n + [z])) for n in node_list]
    return polygon


class EditableTrack(Track):
    """ A track that can be edited through rviz

    This is a horrible hack. Use at your own risk.
    Start by creating a file named `track_under_construction.yaml`
    in `~/.ros` and continue by figuring out how the class
    works by reading the code.
    """

    def __init__(self, *args, **kwargs):
        Track.__init__(self, publish_track=True)
        self.file_name = 'track_under_construction.yaml'
        self.point_topic = '/clicked_point'
        self.set_stay_in_topic = '/initialpose'
        self.set_keep_out_topic = '/move_base_simple/goal'
        self.state = 'Inactive'  # Inactive, StayIn, KeepOut
        self.save_threshold = 0.2

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Track Editing Node: \n"
                      + str(self))
        self._start_listen()
        self._start_publish()
        self._publish_track()
        self._load_polygon()
        rospy.spin()

    def _start_listen(self):
        rospy.Subscriber(self.point_topic,
                         PointStamped,
                         self._on_point_publish)
        rospy.Subscriber(self.set_stay_in_topic,
                         PoseWithCovarianceStamped,
                         self._on_pose_publish)
        rospy.Subscriber(self.set_keep_out_topic,
                         PoseStamped,
                         self._on_goal_publish)

    def _on_point_publish(self, msg):
        point = [msg.point.x, msg.point.y]
        if self.state == 'Inactive':
            pass
        elif self.state == 'StayIn':
            if (len(self._stay_in) > 0
                    and calc_dist(point, self._stay_in[-1]) < self.save_threshold):
                self._save_polygon()
            else:
                self._stay_in.append(point)
        elif self.state == 'KeepOut':
            if (len(self._keep_out) > 0
                    and calc_dist(point, self._keep_out[-1]) < self.save_threshold):
                self._save_polygon()
            else:
                self._keep_out.append(point)
        self._publish_track()

    def _on_pose_publish(self, msg):
        if self.state != 'StayIn':
            self.state = 'StayIn'
        elif len(self._stay_in) > 0:
            position = msg.pose.pose.position
            point = [position.x, position.y]
            self._stay_in = rotate_to_closest(point, self._stay_in)
            del self._stay_in[-1]
        self._publish_track()

    def _on_goal_publish(self, msg):
        if self.state != 'KeepOut':
            self.state = 'KeepOut'
        elif len(self._keep_out) > 0:
            position = msg.pose.position
            point = [position.x, position.y]
            self._keep_out = rotate_to_closest(point, self._keep_out)
            del self._keep_out[-1]
        self._publish_track()

    def _save_polygon(self):
        with open(self.file_name, 'r') as file:
            yaml_dict = load(file)
        if yaml_dict is None:
            yaml_dict = dict()
        if self.state == 'StayIn':
            yaml_dict['stay_in'] = self._stay_in
            rospy.loginfo('stay_in saved to {}'.format(self.file_name))
        elif self.state == 'KeepOut':
            yaml_dict['keep_out'] = self._keep_out
            rospy.loginfo('keep_out saved to {}'.format(self.file_name))
        with open(self.file_name, 'w') as file:
            dump(yaml_dict, file)

    def _load_polygon(self):
        with open(self.file_name, 'r') as file:
            yaml_dict = load(file)
        if yaml_dict is not None:
            stay_in = yaml_dict.get('stay_in', False)
            if stay_in:
                self._stay_in = stay_in
            keep_out = yaml_dict.get('keep_out', False)
            if keep_out:
                self._keep_out = keep_out


def rotate_to_closest(point, point_list):
    if len(point_list) == 0:
        return point_list
    min_distance = float('inf')
    min_ix = 0
    for i, p in enumerate(point_list):
        dist = calc_dist(point, p)
        if dist < min_distance:
            min_distance = dist
            min_ix = i
    return rotate(point_list, min_ix + 1)


def rotate(collection, n):
    return collection[n:] + collection[:n]


def calc_dist(p1, p2):
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
