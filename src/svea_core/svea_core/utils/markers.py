#!/usr/bin/env python
from .. import rosonic as rx
from matplotlib.colors import to_rgba, is_color_like
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import Marker
from rclpy.clock import Clock
import tf_transformations as tf
from builtin_interfaces.msg import Duration

qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class PlaceMarker(rx.Field):
    def __init__(self, **kwds) -> None:
        if namespace := kwds.get('name_space', None):
            self.ns = kwds.get('name_space', None)
        else:
            self.ns = None
    
    def on_startup(self):
        return self

    def marker(self, name:str, color, position, orientation = [0.0, 0.0, 0.0, 0.0], shape = Marker.SPHERE, **kwds):

        topic_name = '/marker/' + name
        mark_pub = self.node.create_publisher(Marker, topic_name, qos_pubber)

        marker = Marker()
        
        marker.header.frame_id = "map"  
        marker.header.stamp = self.node.get_clock().now().to_msg()
        
        marker.type = shape      
        marker.action = Marker.ADD     

        # Position for mark        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        if len(position) > 2:
            marker.pose.position.z = position[2]
        else:
            marker.pose.position.z = 0.0
        
        #Orientation for mark
        if len(orientation) == 1:
            if isinstance(orientation, list):
                orientation = orientation[0]
            x, y, z, w = tf.quaternion_from_euler(ak = orientation)
        elif len(orientation) == 3:
            x, y, z, w = tf.quaternion_from_euler(orientation[0], orientation[1], orientation[2])
        elif len(orientation) == 4:
            x = orientation[0]
            y = orientation[1]
            z = orientation[2]
            w = orientation[3]
        marker.pose.orientation.x = x
        marker.pose.orientation.y = y
        marker.pose.orientation.z = z
        marker.pose.orientation.w = w
        
        # size
        marker.scale.x = 0.5  
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        # Color for mark
        r, g, b, a = parse_color(color)
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        
        # 设置生命周期（0 表示永久）
        marker.lifetime = Duration(sec=1, nanosec=0)  # 1 秒后消失
        
        # 发布 Marker
        mark_pub.publish(marker)


    def traffic_light_marker(self, number=0, position=[0.0,0.0], light_status='Rd'):
        name = f"traffic_{number}"
        position = list(position)
        position.append(0.5)
        position = tuple(position)
        if light_status == 'Rd':
            self.marker(name, "#FF0000", position)
        elif light_status == 'Gr':
            self.marker(name, (0.0, 1.0, 0.0), position)
        else:
            self.marker(name, 'yellow', position)




def parse_color(input_color, default_alpha=1.0):
    """
    Parse color input and return in RGBA format
    
    Args:
        input_color: Input color, can be:
            - Color name string (e.g., 'red', 'blue')
            - Hex string (e.g., '#FF0000', '#FF0000FF')
            - RGB/RGBA tuple (e.g., (255, 0, 0) or (1.0, 0.0, 0.0, 0.5))
        default_alpha: Default alpha value when input color has no alpha channel (0.0-1.0)
    
    Returns:
        RGBA tuple with all components in 0.0-1.0 range
    """
    
    # If input is already in RGBA format (tuple/list with 3 or 4 values), return directly
    if isinstance(input_color, (tuple, list)) and len(input_color) in (3, 4):
        # Check if components are integers in 0-255 range
        if all(isinstance(c, int) and 0 <= c <= 255 for c in input_color[:3]):
            if len(input_color) == 3:
                return input_color[0]/255.0, input_color[1]/255.0, input_color[2]/255.0, default_alpha
            else:
                return input_color[0]/255.0, input_color[1]/255.0, input_color[2]/255.0, input_color[3]/255.0
        # Check if components are floats in 0.0-1.0 range
        elif all(isinstance(c, float) and 0.0 <= c <= 1.0 for c in input_color[:3]):
            if len(input_color) == 3:
                return input_color[0], input_color[1], input_color[2], default_alpha
            else:
                return input_color[0], input_color[1], input_color[2], input_color[3]
    
    # Check if input is a valid color string
    if isinstance(input_color, str):
        if input_color.startswith('#'):
            # Process hex color
            hex_color = input_color.lstrip('#')
            length = len(hex_color)
            
            if length == 3:  # e.g., #RGB
                r = int(hex_color[0]*2, 16) / 255.0
                g = int(hex_color[1]*2, 16) / 255.0
                b = int(hex_color[2]*2, 16) / 255.0
                return r, g, b, default_alpha
            elif length == 6:  # e.g., #RRGGBB
                r = int(hex_color[0:2], 16) / 255.0
                g = int(hex_color[2:4], 16) / 255.0
                b = int(hex_color[4:6], 16) / 255.0
                return r, g, b, default_alpha
            elif length == 8:  # e.g., #RRGGBBAA
                r = int(hex_color[0:2], 16) / 255.0
                g = int(hex_color[2:4], 16) / 255.0
                b = int(hex_color[4:6], 16) / 255.0
                a = int(hex_color[6:8], 16) / 255.0
                return r, g, b, a
        
        # Use matplotlib's color name conversion
        if is_color_like(input_color):
            color = to_rgba(input_color)
            return color[0], color[1], color[2], color[3]
    
    # Raise exception if format is unrecognized
    return 1.0, 1.0, 1.0, 1.0
