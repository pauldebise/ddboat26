"""Test de communication entre bateaux : envoi de position et ecoute.

Le script envoie la position GPS et le cap boussole a 1 Hz au serveur UDP du
lac (communication.py), puis affiche les positions que le serveur renvoie.

Usage :
    python3 test_send_listen.py <ip_serveur> [port] [mon_id]

Sans argument, l'IP, le port et l'id sont lus dans communication.json (les
reglages du panneau Communication de l'IHM) ; sinon le port vaut 6800 et
l'id 3. Exemple :

    python3 test_send_listen.py 192.168.1.42 6800 3

Ctrl-C pour arreter.
"""

from __future__ import print_function

import argparse
import io
import json
import math
import os
import sys
import time

from communication import get_boats, send_position
from drivers.cap_sender_v3 import compute_heading, load_calibration
from drivers.ddboatclass import parser_trame_gll
from drivers.gps_driver_v2 import GpsIO
from drivers.imu9_driver_v2 import Imu9IO

PERIODE = 1.0          # intervalle entre deux envois (s)
TIMEOUT_FIX = 2.0      # attente maximale d'une trame GPS valide (s)

DEFAUT = {"ip": "172.20.26.111", "port": 6800, "mon_id": 18}


# time.monotonic n'existe pas sur les anciens Python du bateau.
maintenant = getattr(time, "monotonic", time.time)


def lire_config():
    """Parametres par defaut : communication.json, sinon valeurs du projet."""
    config = dict(DEFAUT)
    chemin = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "communication.json")
    try:
        with io.open(chemin, encoding="utf-8") as fichier:
            config.update(json.load(fichier))
    except (IOError, OSError, ValueError):
        pass
    return config


def lire_fix(gps, timeout=TIMEOUT_FIX):
    """Attend une trame GLL valide et retourne (lat, lon) en degres decimaux.

    parser_trame_gll verifie le checksum et le statut du fix : une trame
    tronquee ou sans fix est ignoree.
    """
    fin = maintenant() + timeout
    while maintenant() < fin:
        recu, trame = gps.read_non_blocking()
        if recu:
            position = parser_trame_gll(trame.strip())
            if position is not None:
                return position
    return None


def lire_cap(imu, invA, xbar):
    """Cap boussole en degres (0 = Nord, sens horaire)."""
    xmag, ymag, zmag = imu.read_mag_raw()
    return math.degrees(compute_heading([xmag, ymag, zmag], invA, xbar)) % 360.0


def main():
    config = lire_config()
    parseur = argparse.ArgumentParser(
        description="Envoie la position GPS au serveur UDP et affiche les autres bateaux.")
    parseur.add_argument("ip", nargs="?", default=config["ip"],
                         help="IP du serveur (defaut : communication.json)")
    parseur.add_argument("port", nargs="?", type=int, default=int(config["port"]),
                         help="port UDP du serveur (defaut : %(default)s)")
    parseur.add_argument("mon_id", nargs="?", type=int, default=int(config["mon_id"]),
                         help="identifiant de ce bateau (defaut : %(default)s)")
    args = parseur.parse_args()

    if not args.ip:
        parseur.error("IP du serveur absente : la passer en argument ou la "
                      "renseigner dans communication.json")

    print("Serveur {}:{}, bateau {}.".format(args.ip, args.port, args.mon_id))
    print("Envoi toutes les {:.1f} s, Ctrl-C pour arreter.".format(PERIODE))

    gps = GpsIO()
    imu = Imu9IO()
    invA, xbar = load_calibration()

    try:
        while True:
            debut = maintenant()

            position = lire_fix(gps)
            if position is None:
                print("Pas de fix GPS : rien envoye.")
            else:
                lat, lon = position
                cap = lire_cap(imu, invA, xbar)
                send_position(args.ip, args.port, args.mon_id, lat, lon, cap)
                print("Envoi {} : {:.6f}, {:.6f}, cap {:.1f} deg".format(
                    args.mon_id, lat, lon, cap))

            # Laisser au serveur le temps de repondre avant de lire.
            reste = PERIODE - (maintenant() - debut)
            if reste > 0:
                time.sleep(reste)

            bateaux = get_boats()
            if not bateaux:
                print("  Aucune position recue du serveur.")
            for id_, b in sorted(bateaux.items()):
                marque = " (moi)" if id_ == args.mon_id else ""
                print("  Bateau {}{} : {:.6f}, {:.6f}, cap {:.1f} deg, age {:.1f} s".format(
                    id_, marque, b["lat"], b["lon"], b["cap"], b["age"]))
    except KeyboardInterrupt:
        print("\nArret.")
    finally:
        gps.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
