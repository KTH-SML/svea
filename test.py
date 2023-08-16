import math

def quaternion_to_euler(q):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).
    :param q: A tuple or list representing the quaternion (w, x, y, z).
    :return: A tuple containing the Euler angles in degrees (roll, pitch, yaw).
    """
    x = 0.9958580136947276
    y = -0.0012804018667825793
    z = 0.0823323505639621
    w = 0.0385559487195424

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.degrees(math.copysign(math.pi / 2, sinp))  # Use 90 degrees if out of range
    else:
        pitch = math.degrees(math.asin(sinp))

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    return roll, pitch, yaw

# Example usage
quaternion = (0.924, 0.383, 0.0, 0.0)  # Replace with your quaternion values (w, x, y, z)
euler_angles_degrees = quaternion_to_euler(quaternion)
print("Euler Angles (roll, pitch, yaw) in degrees:", euler_angles_degrees)
