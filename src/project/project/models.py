from geometry_msgs.msg import PoseStamped
import math

class Quaternion():
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

class Position():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Orientation():
    def convert_to_quaternion(self):
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        cr = math.cos(self.roll * 0.5)
        sr = math.sin(self.roll * 0.5)
        cp = math.cos(self.pitch * 0.5)
        sp = math.sin(self.pitch * 0.5)

        w = cy * cr * cp + sy * sr * sp
        x = cy * sr * cp - sy * cr * sp
        y = cy * cr * sp + sy * sr * cp
        z = sy * cr * cp - cy * sr * sp
        return Quaternion(x, y, z, w)

    def convert_to_euler(self, quaternion):
        # roll (x-axis rotation)
        sinr_cosp = 2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        cosr_cosp = 1 - 2 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        if (abs(sinp) >= 1):
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (pitch, roll, yaw)

    def __init__(self, pitch, roll, yaw):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw


class Pose():
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation

    def stamped_pose(self, map_frame_id):
        pose = PoseStamped()
        pose.header.frame_id = map_frame_id
        pose.pose.position.x = self.position.x
        pose.pose.position.y = self.position.y
        pose.pose.position.z = self.position.z
        quat = self.orientation.convert_to_quaternion()
        pose.pose.orientation.z = quat.z
        pose.pose.orientation.w = quat.w
        return pose


class Waypoint():
    def __init__(self, name, pose):
        self.pose = pose
        self.name = name
