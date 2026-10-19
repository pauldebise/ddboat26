import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

# Import compatible avec les deux usages :
#   - depuis la racine du dépôt : from drivers.ddboatclass import DDBoat
#   - depuis le dossier drivers/ : from ddboatclass import DDBoat
if __package__:
    from .arduino_driver_v2 import ArduinoIO
    from .gps_driver_v2 import GpsIO
    from .imu9_driver_v2 import Imu9IO
    from .cap_sender_v3 import compute_heading, load_calibration
else:
    from arduino_driver_v2 import ArduinoIO
    from gps_driver_v2 import GpsIO
    from imu9_driver_v2 import Imu9IO
    from cap_sender_v3 import compute_heading, load_calibration

DRIVERS_DIR = Path(__file__).resolve().parent


def angle_diff(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


class DDBoat:


    def __init__(self, ard, gps, imu) -> None:
        self.ard = ard
        self.gps = gps
        self.imu = imu
        self.invA = None
        self.xbar = None

    def calibration(self):
        """Lance les deux acquisitions puis charge/calcule la calibration."""
        scripts = ["create_donnees_mag.py", "create_magNorthWest.py"]

        print("=== Début du processus de calibration du DDBoat ===")

        for script in scripts:
            print(script)
            try:
                subprocess.run(
                    [sys.executable, str(DRIVERS_DIR / script)],
                    check=True,
                    cwd=DRIVERS_DIR,
                )
            except subprocess.CalledProcessError as e:
                print("Une erreur est survenue lors de l'exécution de ")
                print(script)
                return False
            print(script)
            print("exécuté avec succès.")

        # Charge invA.csv / xbar.csv, ou les calcule depuis les CSV bruts.
        self.invA, self.xbar = load_calibration()

        print("=== Calibration terminée avec succès ===")
        return True

    def _ensure_calibration(self):
        if self.invA is None or self.xbar is None:
            self.invA, self.xbar = load_calibration()

    def current_cap_rad(self):
        self._ensure_calibration()
        xmag, ymag, zmag = self.imu.read_mag_raw()
        heading_rad = compute_heading([xmag, ymag, zmag], self.invA, self.xbar)
        return heading_rad % (2 * math.pi)

    def current_cap_deg(self):
        return np.degrees(self.current_cap_rad())

    def tour_complet(self):
        cadran_dir = ["DEVANT", "GAUCHE", "DERRIERE", "DROITE", "DEVANT"]
        cadrans = np.array([0.0, math.pi / 2, math.pi, 3*math.pi / 2])
        cadrans += self.current_cap_rad()

        for i in range(4):

            while True:
                # 1. On récupère le cap actuel du bateau
                current = self.current_cap_rad()

                VITESSE_ROTATION = 60
                print(current)
                if current > cadrans[i]:
                    print("Cadrant", cadran_dir[i], cadran_dir[i+1], "atteint avec succès !")
                    break
                else:
                    self.ard.send_arduino_cmd_motor(-VITESSE_ROTATION, VITESSE_ROTATION)

                # Petite pause pour ne pas saturer le processeur du Raspberry Pi
                time.sleep(0.05)


            # Petite pause entre chaque quadrant si besoin de stabilisation
            time.sleep(0.5)
        self.ard.send_arduino_cmd_motor(0, 0)



if __name__ == "__main__":
    from imu9_driver_v2 import Imu9IO
    from gps_driver_v2 import GpsIO
    from arduino_driver_v2 import ArduinoIO
    ard = ArduinoIO()
    gps = GpsIO()
    imu = Imu9IO()
    ddb = DDBoat(ard, gps, imu)
    
    ddb.tour_complet()