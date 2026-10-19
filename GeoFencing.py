from drivers.arduino_driver_v2 import ArduinoIO
from drivers.gps_driver_v2 import GpsIO
from drivers.imu9_driver_v2 import Imu9IO
from drivers.ddboatclass import DDBoat
import numpy as np



def gradient_artificiel_point(x_pt: float, y_pt: float, x_pos: float, y_pos: float, attract: bool):
    signe = (-1) if attract else 1
    return signe * np.array([x_pos-x_pt, y_pos-y_pt])
    

def gradient_artificiel_ligne(x_n: float, y_n: float, x_pt: float, y_pt: float, x_pos: float, y_pos: float):
    n = np.array([x_n, y_n])
    Q = n@n.T
    pp = np.array([x_pos-x_pt, y_pos-x_pt])
    return -2 * Q@pp

def gradient_artificiel_uniforme(x_v, y_v):
    return np.array([x_v, y_v])


def main(ddboat: DDBoat):
    pass


if __name__ == "__main__":
    ard = ArduinoIO
    gps = GpsIO
    imu = Imu9IO
    ddboat = DDBoat(ard, gps, imu)
    main(ddboat)