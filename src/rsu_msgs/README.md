# ROS Messages for RSU Platform

Custom ROS messages for object detection/tracking and more, between road side
unit (RSU) and other connected units.

The repository does not contain any source code except the message type
definitions.


## Object Messages

- `Object.msg` - Fields for object classification, the
region-of-interest on the image, object tracking id, classification and
tracking confidence
- `ObjectPose.msg` - Fields for `Object.msg` and pose for object position in
real-world.
- `StampedObjectArray.msg` - Multiple objects are identified per frame, here we
have a time-stamped array of objects for that purpose.
- `StampedObjectPoseArray.msg` - Same for `StampedObjectArray.msg` but using
`ObjectPose.msg` instead of `Object.msg`.
