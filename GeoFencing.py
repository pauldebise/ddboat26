from drivers.arduino_driver_v2 import ArduinoIO
from drivers.gps_driver_v2 import GpsIO
from drivers.imu9_driver_v2 import Imu9IO
from drivers.ddboatclass import DDBoat



def main(ddboat: DDBoat):
    pass


if __name__ == "__main__":
    ard = ArduinoIO
    gps = GpsIO
    imu = Imu9IO
    ddboat = DDBoat(ard, gps, imu)
    main(ddboat)