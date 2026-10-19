from drivers.arduino_driver_v2 import ArduinoIO
from drivers.gps_driver_v2 import GpsIO
from drivers.imu9_driver_v2 import Imu9IO

class DDBoat:

    def __init__(self, ard, gps, imu) -> None:
        self.ard: ArduinoIO = ard
        self.gps: GpsIO = gps
        self.imu: Imu9IO = imu

