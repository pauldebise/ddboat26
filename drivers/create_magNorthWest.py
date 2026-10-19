# -*- coding: utf-8 -*-
"""
create_magNorthWest.py — A lancer sur le DD-Boat
-------------------------------------------------
Lit les valeurs brutes du magnétomètre dans deux orientations
(Nord puis Ouest) et crée le fichier magNorthWest.csv.

DEUXIEME ETAPE DE CALIBRATION
Ce script sert à orienter le référentiel magnétique vers le Nord géographique —
c'est la deuxième étape de la calibration, après create_donnees_mag.py.

Usage :
  python create_magNorthWest.py
"""

import sys
import time
import numpy as np

sys.path.append('/home/ue32/drivers-ddboat-v2')
from imu9_driver_v2 import Imu9IO

N_SAMPLES = 20  # nombre de mesures moyennées pour réduire le bruit


def read_average_mag(imu, n=N_SAMPLES):
    """Prend n mesures et retourne la moyenne pour limiter le bruit."""
    samples = []
    for _ in range(n):
        samples.append(imu.read_mag_raw())
        time.sleep(0.05)
    arr = np.array(samples)
    return arr.mean(axis=0)


if __name__ == "__main__":
    imu = Imu9IO()

    print("=" * 50)
    print("  Création de magNorthWest.csv")
    print("=" * 50)

    # ── Mesure Nord ──────────────────────────────────────────────────────────
    print("\n[ETAPE 1/2] Oriente le DD-Boat vers le NORD.")
    print("  (Utilise une boussole ou ton téléphone)")
    input("  → Appuie sur ENTRÉE quand le bateau est bien orienté Nord...")

    print("  Lecture en cours ({} mesures)...".format(N_SAMPLES))
    xnorth = read_average_mag(imu)
    print("  xnorth = {}".format(xnorth))

    # ── Mesure Ouest ─────────────────────────────────────────────────────────
    print("\n[ETAPE 2/2] Oriente le DD-Boat vers l'OUEST.")
    input("  → Appuie sur ENTRÉE quand le bateau est bien orienté Ouest...")

    print("  Lecture en cours ({} mesures)...".format(N_SAMPLES))
    xwest = read_average_mag(imu)
    print("  xwest  = {}".format(xwest))

    # ── Sauvegarde ───────────────────────────────────────────────────────────
    data = np.concatenate([xnorth, xwest])
    np.savetxt('magNorthWest.csv', data)

    print("\n[OK] magNorthWest.csv créé avec succès !")
    print("     Tu peux maintenant lancer cap_sender_v3.py")
