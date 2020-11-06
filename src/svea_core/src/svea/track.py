"""
Track handling module. Contains classes for handling tracks.
"""

from threading import Thread
from copy import deepcopy
import rospy
from geometry_msgs.msg import (PolygonStamped,
                               Point32,
                               PointStamped,
                               PoseWithCovarianceStamped,
                               PoseStamped)
try:
    from yaml import load, dump
    yaml_available = True
except ImportError:
    yaml_available = False
if yaml_available:
    try:
        from yaml import CLoader as Loader, CDumper as Dumper
    except ImportError:
        from yaml import Loader, Dumper


__license__ = "MIT"
__maintainer__ = "Tobias Bolin"
__email__ = "tbolin@kth.se"
__status__ = "Development"


class Track(object):
    """ Representation of a track.
    Useful for loading the track parameters.
    Can publish the track as polygons for visualization in rviz.

    The track is defined by two polygons.
    One stay inside polygon and one keep out polygon.
    The coordinates are defined relative to a given occupation grid map.
    The polygons can be assumed to be simple (ie. not self intersecting).
    There are however no check for self intersections done by this class.

    This track definition is intended as a unified way of defining
    track boundaries in a way that is easy to relate to the real world.
    It is not necesarily ideal for direct use in path planning and similar tasks.

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
        rospy.logdebug('Track published')

    def _get_stay_in_from_parameters(self):
        stay_in = rospy.search_param('stay_in')
        if stay_in is not None:
            self._stay_in = rospy.get_param(stay_in)
        else:
            track = rospy.search_param('track')
            self._stay_in = rospy.get_param(track)[0]

    def _get_keep_out_from_parameters(self):
        keep_out = rospy.search_param('keep_out')
        if keep_out is not None:
            self._keep_out = rospy.get_param(keep_out)
        else:
            track = rospy.search_param('track')
            self._keep_out = rospy.get_param(track)[1]

    @property
    def stay_in(self):
        """The stay inside polygon, guaranteed to be closed"""
        if self._stay_in is None:
            self._get_stay_in_from_parameters()
        return close_polygon(self._stay_in)

    @property
    def keep_out(self):
        """The keep out polygon, guaranteed to be closed"""
        if self._keep_out is None:
            self._get_keep_out_from_parameters()
        return close_polygon(self._keep_out)


def list_to_polygon(node_list, z=0.1):
    polygon = [Point32(*(n + [z])) for n in node_list]
    return polygon


def close_polygon(polygon_list):
    polygon_list = deepcopy(polygon_list)
    if polygon_list[0] != polygon_list[-1]:
        polygon_list.append(polygon_list[0])
    return polygon_list


class EditableTrack(Track):
    """ A track that can be edited through rviz

    This is a horrible hack. Use at your own risk.
    Start by creating a file named `track_under_construction.yaml`
    in `~/.ros` and runing rosdep.
    Continue by figuring out how the class works by reading the code.
    """

    def __init__(self, *args, **kwargs):
        Track.__init__(self, publish_track=True)
        self.file_name = 'track_under_construction.yaml'
        self.point_topic = '/clicked_point'
        self.set_stay_in_topic = '/initialpose'
        self.set_keep_out_topic = '/move_base_simple/goal'
        self.state = 'inactive'  # inactive, stay_in, keep_out
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
        if self.state == 'inactive':
            pass
        elif self.state == 'stay_in':
            if (len(self._stay_in) > 0
                    and calc_dist(point, self._stay_in[-1]) < self.save_threshold):
                self._save_polygon()
            else:
                self._stay_in.append(point)
        elif self.state == 'keep_out':
            if (len(self._keep_out) > 0
                    and calc_dist(point, self._keep_out[-1]) < self.save_threshold):
                self._save_polygon()
            else:
                self._keep_out.append(point)
        self._publish_track()

    def _handle_polygon_select(self, position, polygon_name):
        point = [position.x, position.y]
        var_name = '_' + polygon_name
        clean_name = polygon_name.replace('_', ' ')
        polygon = getattr(self, var_name)
        polygon = rotate_to_closest(point, polygon)
        if self.state == polygon_name and len(polygon) > 0:
            rospy.loginfo("Removed point at {0} from {1} polygon"
                          .format(polygon[-1], clean_name))
            del polygon[-1]
        if len(polygon) > 0:
            rospy.loginfo("Now adding to {0} from point {1}"
                          .format(clean_name, polygon[-1]))
        else:
            rospy.loginfo("Now adding to {0}".format(clean_name))
        self.state = polygon_name
        setattr(self, var_name, polygon)
        self._publish_track()

    def _on_pose_publish(self, msg):
        position = msg.pose.pose.position
        self._handle_polygon_select(position, 'stay_in')

    def _on_goal_publish(self, msg):
        position = msg.pose.position
        self._handle_polygon_select(position, 'keep_out')

    def _save_polygon(self):
        with open(self.file_name, 'r') as file:
            yaml_dict = load(file, Loader=Loader)
        if yaml_dict is None:
            yaml_dict = dict()
        if self.state == 'stay_in':
            yaml_dict['stay_in'] = self._stay_in
            rospy.loginfo('stay_in saved to {}'.format(self.file_name))
        elif self.state == 'keep_out':
            yaml_dict['keep_out'] = self._keep_out
            rospy.loginfo('keep_out saved to {}'.format(self.file_name))
        with open(self.file_name, 'w') as file:
            dump(yaml_dict, file, Dumper=Dumper)

    def _load_polygon(self):
        with open(self.file_name, 'r') as file:
            yaml_dict = load(file, Loader=Loader)
        if yaml_dict is not None:
            stay_in = yaml_dict.get('stay_in', False)
            if stay_in:
                self._stay_in = stay_in
            keep_out = yaml_dict.get('keep_out', False)
            if keep_out:
                self._keep_out = keep_out


def rotate_to_closest(point, point_list):
    """Rotate a list so that the element closest to point is at the end of the list

    :param point: a 2D point in space, represented by a list or tuple containing 2 floats 
    :type point: list or tuple
    :param point_list: A list of 2D points
    :type point_list: A list of points
    :returns: The rotated point list
    :rtype: A list of points
    """
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
