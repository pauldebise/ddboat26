#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mission : sortie de zone, detection, tour complet, retour a la base.

Deroulement
-----------
1. Attente d'un fix GPS (le bateau est pose a la base).
2. SORTIE : le bateau part perpendiculairement a la bordure la plus proche
   (ou a celle choisie avec --cote), jusqu'a etre hors limites.
3. DETECTION : la sortie est confirmee sur plusieurs fix GPS successifs.
4. TOUR : tour complet de 360 degres.
5. RETOUR : retour a la base par le couloir (Environnement.direction()).

Pendant toute la mission, l'etat du bateau est envoye a l'IHM par UDP.

Usage (sur le bateau)
---------------------
    # Terminal 1 : l'IHM, en ecoute de la telemetrie
    python3 ihm_lac.py --mission --host 0.0.0.0 --no-browser

    # Terminal 2 : la mission
    python3 mission_sortie_retour.py

Puis ouvrir http://<IP_BATEAU>:8000/ depuis l'ordinateur.
L'IHM peut aussi tourner sur l'ordinateur : lancer la mission avec
--ihm-ip <IP_ORDINATEUR>.

Ctrl+C coupe les moteurs et arrete la mission.
"""

import argparse
import json
import math
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

BASE_DIR = Path(__file__).resolve().parent
DRIVERS_DIR = BASE_DIR / "drivers"
if str(DRIVERS_DIR) not in sys.path:
    sys.path.insert(0, str(DRIVERS_DIR))

from ddboatclass import DDBoat, dist_cart  # noqa: E402
from environnementclass import Environnement  # noqa: E402

AGE_FIX_MAX = 3.0  # au-dela, la position GPS est consideree perimee (s)


class Telemetrie(object):
    """Envoie l'etat du bateau a l'IHM par UDP, dans un thread separe.

    Le thread lit seulement le GPS (protege par un verrou dans DDBoat) et
    des valeurs deja mesurees : il ne touche ni a l'IMU ni aux moteurs.
    L'envoi continue donc pendant le tour complet.
    """

    def __init__(self, ddb, env, hote, port, periode=0.25):
        self.ddb = ddb
        self.env = env
        self.adresse = (hote, port)
        self.periode = periode
        self.phase = "INIT"
        self.message = ""
        self.verrou = threading.Lock()
        self.stop = threading.Event()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.thread = threading.Thread(target=self._boucle, name="telemetrie")
        self.thread.daemon = True

    def demarrer(self):
        self.thread.start()

    def definir_phase(self, phase, message=""):
        with self.verrou:
            self.phase = phase
            self.message = message
        print("[{}] {}".format(phase, message))
        self.envoyer()

    def definir_message(self, message):
        with self.verrou:
            self.message = message

    def _moteurs(self):
        ard = self.ddb.ard
        try:
            gauche = -ard.cmdl if ard.dirl == "-" else ard.cmdl
            droite = -ard.cmdr if ard.dirr == "-" else ard.cmdr
            return [int(gauche), int(droite)]
        except AttributeError:
            return None

    def paquet(self):
        with self.verrou:
            phase = self.phase
            message = self.message

        position = self.ddb.get_DDboat_cart_position(
            self.env.home_latlong, self.env.rotation)
        cap = self.ddb.dernier_cap
        cap_cible = self.ddb.dernier_cap_cible

        donnees = {
            "source": "ddboat-mission",
            "t": time.time(),
            "phase": phase,
            "message": message,
            "x": None,
            "y": None,
            "lat": None,
            "lon": None,
            "fix_age": self.ddb.age_fix_gps(),
            "dans_zone": None,
            "distance": None,
            "cap": math.degrees(cap) if cap is not None else None,
            "cap_cible": math.degrees(cap_cible) if cap_cible is not None else None,
            # Orientation pour le dessin : angle mathematique (0 = axe x,
            # sens anti-horaire) dans le repere local du lac.
            "heading": (math.pi / 2 - cap - self.env.rotation) if cap is not None else None,
            "moteurs": self._moteurs(),
        }
        if position is not None:
            x, y = float(position[0]), float(position[1])
            lat, lon = self.ddb.derniere_position_gps
            donnees.update({
                "x": x,
                "y": y,
                "lat": lat,
                "lon": lon,
                "dans_zone": bool(self.env.lim(x, y)),
                "distance": dist_cart((x, y), self.env.home),
            })
        return donnees

    def envoyer(self):
        try:
            corps = json.dumps(self.paquet()).encode("utf-8")
            self.sock.sendto(corps, self.adresse)
        except Exception:
            # La telemetrie ne doit jamais interrompre la mission.
            pass

    def _boucle(self):
        while not self.stop.wait(self.periode):
            self.envoyer()

    def fermer(self):
        self.stop.set()
        self.envoyer()
        self.sock.close()


def normale_exterieure(env, cote, position):
    """Vecteur unitaire perpendiculaire a la bordure, vers l'exterieur.

    Retourne (normale, distance) ou distance est l'ecart entre la position
    et la droite de bordure.
    """
    bordure = env.bordure_droite if cote == "droite" else env.bordure_gauche
    a = np.array(bordure[0], dtype=float)
    b = np.array(bordure[1], dtype=float)
    v = (b - a) / np.linalg.norm(b - a)
    normale = np.array([-v[1], v[0]])
    # Le bateau est a l'interieur : la normale doit aller du bateau vers la
    # bordure.
    if np.dot(a - position, normale) < 0.0:
        normale = -normale
    distance = float(np.dot(a - position, normale))
    return normale, distance


def choisir_direction_sortie(env, position, cote, cap_sortie):
    """Retourne (vecteur local, description) de la direction de sortie."""
    if cap_sortie is not None:
        # Cap boussole impose -> vecteur dans le repere local.
        angle = math.pi / 2 - math.radians(cap_sortie) - env.rotation
        return np.array([math.cos(angle), math.sin(angle)]), \
            "cap impose {:.0f} deg".format(cap_sortie)

    if cote == "auto":
        _, d_droite = normale_exterieure(env, "droite", position)
        _, d_gauche = normale_exterieure(env, "gauche", position)
        cote = "droite" if d_droite <= d_gauche else "gauche"

    normale, distance = normale_exterieure(env, cote, position)
    return normale, "bordure {} a {:.0f} m".format(cote, distance)


def phase_sortie(ddb, env, telem, direction, confirmations, timeout,
                 distance_max):
    """Navigue dans `direction` jusqu'a une sortie de zone confirmee.

    Retourne True si la sortie est detectee, False sinon (timeout,
    distance maximale).
    """
    debut = time.monotonic()
    hors_consecutifs = 0
    dernier_fix = ddb.compteur_fix_gps
    try:
        while True:
            if time.monotonic() - debut > timeout:
                print("Sortie : temps maximal depasse.")
                return False

            position = ddb.get_DDboat_cart_position(env.home_latlong, env.rotation)
            age = ddb.age_fix_gps()
            if position is None or age is None or age > AGE_FIX_MAX:
                ddb._arreter_moteurs()
                telem.definir_message("GPS perdu, moteurs coupes")
                time.sleep(0.2)
                continue

            x, y = position
            if dist_cart((x, y), env.home) > distance_max:
                print("Sortie : distance maximale depassee.")
                return False

            # La detection ne compte que les nouveaux fix : une meme
            # position relue plusieurs fois ne doit pas valider la sortie.
            if ddb.compteur_fix_gps != dernier_fix:
                dernier_fix = ddb.compteur_fix_gps
                if env.lim(x, y):
                    hors_consecutifs = 0
                    telem.definir_message("")
                else:
                    hors_consecutifs += 1
                    telem.definir_message("confirmation {}/{}".format(
                        hors_consecutifs, confirmations))
                    if hors_consecutifs >= confirmations:
                        return True

            ddb.maintien_cap(direction, env.rotation)
            time.sleep(0.05)
    finally:
        ddb._arreter_moteurs()


def parser_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="Sortie de zone, tour complet puis retour a la base.")
    parser.add_argument("--cote", choices=("auto", "droite", "gauche"),
                        default="auto",
                        help="bordure a franchir (defaut : la plus proche)")
    parser.add_argument("--cap-sortie", type=float, default=None,
                        help="impose un cap boussole de sortie en degres")
    parser.add_argument("--confirmations", type=int, default=3,
                        help="fix GPS hors limites successifs pour valider "
                             "la sortie (defaut : 3)")
    parser.add_argument("--timeout-sortie", type=float, default=300.0,
                        help="duree maximale de la sortie en s (defaut : 300)")
    parser.add_argument("--distance-max", type=float, default=300.0,
                        help="distance maximale a la base pendant la sortie "
                             "en m (defaut : 300)")
    parser.add_argument("--timeout-retour", type=float, default=900.0,
                        help="duree maximale du retour en s (defaut : 900)")
    parser.add_argument("--rayon-arrivee", type=float, default=5.0,
                        help="distance a la base consideree comme atteinte "
                             "en m (defaut : 5)")
    parser.add_argument("--gps", type=int, choices=(0, 1, 2), default=0,
                        help="peripherique GPS (defaut : 0)")
    parser.add_argument("--ihm-ip", default="127.0.0.1",
                        help="adresse de l'IHM (defaut : 127.0.0.1)")
    parser.add_argument("--ihm-port", type=int, default=5006,
                        help="port UDP de l'IHM (defaut : 5006)")
    return parser.parse_args(argv)


def main(argv=None):
    args = parser_arguments(argv)

    from arduino_driver_v2 import ArduinoIO
    from gps_driver_v2 import GpsIO
    from imu9_driver_v2 import Imu9IO

    ard = ArduinoIO()
    gps = GpsIO(tty_dev=args.gps)
    imu = Imu9IO()
    ddb = DDBoat(ard, gps, imu)
    env = Environnement()

    telem = Telemetrie(ddb, env, args.ihm_ip, args.ihm_port)
    telem.demarrer()
    print("Telemetrie envoyee vers {}:{} (UDP).".format(args.ihm_ip, args.ihm_port))

    try:
        # Lecture du cap pour l'IHM avant le depart.
        ddb.current_cap_rad()

        telem.definir_phase("ATTENTE_GPS", "en attente d'un fix GPS")
        position = ddb.attendre_position(env.home_latlong, env.rotation)
        if position is None:
            telem.definir_phase("ECHEC", "aucun fix GPS")
            return 1

        position = np.array(position, dtype=float)
        distance_base = dist_cart(position, env.home)
        if distance_base > 15.0:
            print("Attention : le bateau est a {:.0f} m de la base.".format(distance_base))

        sortie_ok = True
        if env.lim(position[0], position[1]):
            direction, description = choisir_direction_sortie(
                env, position, args.cote, args.cap_sortie)
            telem.definir_phase("SORTIE", description)
            sortie_ok = phase_sortie(ddb, env, telem, direction,
                                     args.confirmations, args.timeout_sortie,
                                     args.distance_max)
        else:
            print("Le bateau est deja hors limites au depart.")

        if sortie_ok:
            telem.definir_phase("HORS_LIMITES", "sortie de zone detectee")
            time.sleep(1.0)
            telem.definir_phase("TOUR", "tour complet")
            ddb.tour_complet()
        else:
            telem.definir_phase("RETOUR", "sortie abandonnee, retour direct")

        telem.definir_phase("RETOUR", "retour par le couloir")
        arrive = ddb.homing(env, rayon_arrivee=args.rayon_arrivee,
                            timeout=args.timeout_retour, age_fix_max=AGE_FIX_MAX)
        if arrive:
            telem.definir_phase("ARRIVEE", "base atteinte")
            return 0
        telem.definir_phase("ECHEC", "retour non termine")
        return 1
    finally:
        ddb._arreter_moteurs()
        telem.fermer()
        try:
            gps.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
