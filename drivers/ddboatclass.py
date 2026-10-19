from drivers.arduino_driver_v2 import ArduinoIO
from drivers.gps_driver_v2 import GpsIO
from drivers.imu9_driver_v2 import Imu9IO

class DDBoat:

    def __init__(self, ard: ArduinoIO, gps: GpsIO, imu: Imu9IO) -> None:
        self.ard = ard
        self.gps = gps
        self.imu = imu

