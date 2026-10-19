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


    VITESSE_ROTATION = 60               # consigne moteur (0-255)
    TOLERANCE_CAP = math.radians(10.0)  # ecart accepte autour du cap cible
    TIMEOUT_PALIER = 15.0               # duree max pour atteindre un cap (s)
    TEMPS_STABILISATION = 0.00          # pause moteurs arretes entre deux caps (s)
    # Sens du tour : +1 = horaire (defaut), -1 = anti-horaire.
    SENS_TOUR = 1
    # Polarite des moteurs : +1 si (-VITESSE, +VITESSE) augmente le cap
    # mesure, -1 si cette commande le fait diminuer. Le cap du capteur
    # augmentant dans le sens horaire, la valeur par defaut est -1.
    SENS_ROTATION = -1

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

    def _arreter_moteurs(self):
        """Coupe les deux moteurs."""
        self.ard.send_arduino_cmd_motor(0, 0)

    def _tourner_vers_cap(self, cible):
        """Fait tourner le bateau jusqu'au cap cible.

        Retourne True si le cap est atteint, False si le temps maximum
        est depasse.
        """
        cible %= 2 * math.pi
        debut = time.monotonic()

        while True:
            current = self.current_cap_rad()

            # angle_diff() donne l'ecart le plus court entre les deux caps,
            # dans l'intervalle [-pi, pi]. C'est indispensable car les angles
            # rebouclent a 2*pi (359 degres est proche de 1 degre).
            erreur = angle_diff(cible, current)

            if abs(erreur) <= self.TOLERANCE_CAP:
                self._arreter_moteurs()
                print("Cap atteint : {:.1f} deg (ecart {:.1f} deg)".format(
                    math.degrees(current), math.degrees(erreur)))
                return True

            if time.monotonic() - debut > self.TIMEOUT_PALIER:
                self._arreter_moteurs()
                print("Echec : cap cible {:.1f} deg, cap actuel {:.1f} deg".format(
                    math.degrees(cible), math.degrees(current)))
                return False

            # Le cap du capteur augmente dans le sens horaire.
            # Une erreur positive signifie donc qu'il faut tourner a droite.
            # SENS_ROTATION adapte la commande au sens physique des moteurs.
            if erreur * self.SENS_ROTATION > 0:
                self.ard.send_arduino_cmd_motor(-self.VITESSE_ROTATION,
                                                self.VITESSE_ROTATION)
            else:
                self.ard.send_arduino_cmd_motor(self.VITESSE_ROTATION,
                                                -self.VITESSE_ROTATION)

            # Petite pause pour ne pas saturer le processeur du Raspberry Pi.
            time.sleep(0.05)

    def tour_complet(self):
        """Fait faire au bateau un tour complet de 360 degres.

        Par defaut (SENS_TOUR = 1), le bateau tourne dans le sens horaire :
        cap de depart, +90 (droite), +180 (derriere), +270 (gauche), puis
        +360 degres (retour au cap initial).
        """
        cap_initial = self.current_cap_rad()
        offsets = [0.0, math.pi / 2, math.pi, 3 * math.pi / 2, 2 * math.pi]

        # Sens horaire par defaut : +90 = droite, +180 = derriere,
        # +270 = gauche, +360 = retour au cap initial.
        if self.SENS_TOUR > 0:
            cadran_dir = ["DEVANT", "DROITE", "DERRIERE", "GAUCHE", "DEVANT"]
        else:
            offsets = [-offset for offset in offsets]
            cadran_dir = ["DEVANT", "GAUCHE", "DERRIERE", "DROITE", "DEVANT"]

        print("=== Debut du tour complet ===")
        print("Cap initial : {:.1f} deg".format(math.degrees(cap_initial)))

        try:
            for i, offset in enumerate(offsets):
                cible = (cap_initial + offset) % (2 * math.pi)
                print("--> Cap cible {} : {:.1f} deg".format(
                    cadran_dir[i], math.degrees(cible)))

                if not self._tourner_vers_cap(cible):
                    print("Tour complet interrompu.")
                    return False

                # On laisse le bateau se stabiliser, moteurs arretes, avant
                # de mesurer le cap suivant.
                time.sleep(self.TEMPS_STABILISATION)

            print("=== Tour complet termine ===")
            return True
        finally:
            # Securite : on coupe les moteurs dans tous les cas, meme si une
            # exception survient pendant la rotation.
            self._arreter_moteurs()



if __name__ == "__main__":
    from imu9_driver_v2 import Imu9IO
    from gps_driver_v2 import GpsIO
    from arduino_driver_v2 import ArduinoIO
    ard = ArduinoIO()
    gps = GpsIO()
    imu = Imu9IO()
    ddb = DDBoat(ard, gps, imu)
    
    ddb.tour_complet()