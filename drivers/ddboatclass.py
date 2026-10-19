from arduino_driver_v2 import ArduinoIO
from gps_driver_v2 import GpsIO
from imu9_driver_v2 import Imu9IO
from cap_sender_v3 import compute_heading, load_calibration
import math
import time
import subprocess
import sys
import numpy as np



def angle_diff(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))



class DDBoat:

    def __init__(self, ard, gps, imu) -> None:
        self.ard: ArduinoIO = ard
        self.gps: GpsIO = gps
        self.imu: Imu9IO = imu

    def calibration(self):
        #Appeler les 3 fichiers de calibrations
        scripts = ["create_donnees_mag.py", "create_magNorthWest.py", "cap_sender_v3.py"]

        print("=== Début du processus de calibration du DDBoat ===")

        for script in scripts:
            print(f"--> Exécution de {script}...")
            try:
                # Exécute le script avec l'interpréteur Python actuel (sys.executable)
                resultat = subprocess.run([sys.executable, script], check=True, text=True)
                print(f" {script} exécuté avec succès.")
            except subprocess.CalledProcessError as e:
                print(f" Une erreur est survenue lors de l'exécution de {script} : {e}")
                return False

            print("=== Calibration terminée avec succès ===")
            return True
        #attribuer les invA et xbar à l'objet

        self.invA = np.loadtxt("invA.csv", delimiter=",")
        self.xbar = np.loadtxt("xbar.csv", delimiter=",")



    def current_cap_rad(self):
        xmag, ymag, zmag = self.imu.read_mag_raw()
        heading_rad = compute_heading([xmag, ymag, zmag], self.invA, self.xbar)
        return heading_rad % 2*math.pi

    def current_cap_deg(self):
         return np.degrees(self.current_cap_rad())


    def tour_complet(self):
        cadrans = np.array([0.0, math.pi / 2, math.pi, -math.pi / 2])
        tolerance = 0.1  # Tolérance d'acceptation du cap (en radians, ~6 degrés)
        cadrans+=self.current_cap_rad()
        for target in cadrans:
            print(f"-> Rotation vers le cap cible : {target:.2f} rad")

            while True:
                # 1. On récupère le cap actuel du bateau
                current = compute_heading()

                # 2. On calcule l'écart angulaire
                erreur = angle_diff(target, current)

                # 3. Si on est assez proche du cap cible, on sort de la boucle
                if abs(erreur) < tolerance:
                    print(f"Cap {target:.2f} atteint avec succès !")
                    # stop_moteurs() # Décommentez selon vos fonctions de pilotage
                    break

            # 4. Commande des moteurs proportionnelle ou en tout-ou-rien pour tourner
            if erreur > 0:
                self.ard.send_arduino_cmd_motor()
                # Tourner dans un sens (ex: à gauche)
                # Exemple : set_motor_speeds(-vitesse_rotation, vitesse_rotation)
                pass
            else:
                # Tourner dans l'autre sens (ex: à droite)
                # Exemple : set_motor_speeds(vitesse_rotation, -vitesse_rotation)
                pass

            # Petite pause pour ne pas saturer le processeur du Raspberry Pi
            time.sleep(0.05)

        # Petite pause entre chaque quadrant si besoin de stabilisation
        time.sleep(0.5)


