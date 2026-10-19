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

    VITESSE_ROTATION = 60  # consigne moteur utilisée par tour_complet (0-255)

    def __init__(self, ard, gps, imu) -> None:
        self.ard: ArduinoIO = ard
        self.gps: GpsIO = gps
        self.imu: Imu9IO = imu
        self.invA = None
        self.xbar = None

    def calibration(self):
        """Lance les deux acquisitions puis charge/calcule la calibration."""
        # cap_sender_v3.py n'est volontairement pas exécuté ici : c'est un
        # programme de diffusion UDP qui tourne en boucle. Ses fonctions
        # load_calibration / compute_heading sont importées plus haut.
        scripts = ["create_donnees_mag.py", "create_magNorthWest.py"]

        print("=== Début du processus de calibration du DDBoat ===")

        for script in scripts:
            print(f"--> Exécution de {script}...")
            try:
                subprocess.run(
                    [sys.executable, str(DRIVERS_DIR / script)],
                    check=True,
                    cwd=DRIVERS_DIR,
                )
            except subprocess.CalledProcessError as e:
                print(f"Une erreur est survenue lors de l'exécution de {script} : {e}")
                return False
            print(f"    {script} exécuté avec succès.")

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
        cadrans = np.array([0.0, math.pi / 2, math.pi, -math.pi / 2])
        tolerance = 0.1  # tolérance d'acceptation du cap (radians, ~6 degrés)
        cadrans += self.current_cap_rad()

        for target in cadrans:
            target %= 2 * math.pi
            print(f"-> Rotation vers le cap cible : {target:.2f} rad")

            while True:
                # 1. On récupère le cap actuel du bateau
                current = self.current_cap_rad()

                # 2. On calcule l'écart angulaire
                erreur = angle_diff(target, current)

                # 3. Si on est assez proche du cap cible, on sort de la boucle
                if abs(erreur) < tolerance:
                    print(f"Cap {target:.2f} atteint avec succès !")
                    break

                # 4. Commande des moteurs en tout-ou-rien pour tourner
                # (inverser les signes si le bateau tourne dans le mauvais sens)
                if erreur > 0:
                    self.ard.send_arduino_cmd_motor(-self.VITESSE_ROTATION, self.VITESSE_ROTATION)
                else:
                    self.ard.send_arduino_cmd_motor(self.VITESSE_ROTATION, -self.VITESSE_ROTATION)

                # Petite pause pour ne pas saturer le processeur du Raspberry Pi
                time.sleep(0.05)

            self.ard.send_arduino_cmd_motor(0, 0)

            # Petite pause entre chaque quadrant si besoin de stabilisation
            time.sleep(0.5)
