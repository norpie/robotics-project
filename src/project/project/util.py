import math


class LidarData:
    def __init__(self, msg):
        self._ranges = msg.ranges

    def index_per_degree(self):
        return len(self._ranges)/360.0

    def degree_per_index(self):
        return 360.0/len(self._ranges)

    def min(self, degree_min, degree_max):
        index_per_degree_amount = self.index_per_degree()
        max_index = math.ceil(index_per_degree_amount * degree_max)
        min_index = math.ceil(index_per_degree_amount * degree_min)
        if max_index > len(self._ranges) - 1:
            max_index = len(self._ranges) - 1
        return min(self._ranges[min_index:max_index])

    def at(self, degree):
        index = math.ceil(self.index_per_degree() * degree)
        if index > len(self._ranges) - 1:
            index = len(self._ranges) - 1
        return self._ranges[index]


class OdomData:
    def __init__(self, msg):
        self.orientation = msg.pose.pose.orientation
        self.position = msg.pose.pose.position
        self.x = self.position.x
        self.y = self.position.y
        self.z = self.position.z
        (self.roll, self.pitch, self.yaw) = to_euler(self.orientation.x,
                                                     self.orientation.y,
                                                     self.orientation.z,
                                                     self.orientation.w)
        self.degrees = convert_to_360(math.degrees(self.yaw))


# https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def to_euler(x, y, z, w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = math.sqrt(1 + 2 * (w * y - x * z))
    cosp = math.sqrt(1 - 2 * (w * y - x * z))
    pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)


def signed_rotation(source, target):
    a = target - source
    a = (a + 180) % 360 - 180
    return a


def map(x):
    x_min = 0.0
    x_max = 20.0
    y_min = 0.05
    y_max = 0.2

    return ((x - x_min) * (y_max - y_min) / (x_max - x_min)) + y_min


def convert_to_360(angle):
    if angle < 0:
        return angle + 360
    return angle
