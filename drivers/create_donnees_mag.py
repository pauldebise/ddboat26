# -*- coding: utf-8 -*-
"""
create_donnees_mag.py — A lancer sur le DD-Boat
------------------------------------------------
Calibration du magnétomètre, puis sauvegarde dans donnees_mag.csv.

Ce fichier est ensuite utilisé par cap_sender_v3.py dans compute_calibration()
pour fitter une ellipsoïde sur ces points et calculer la matrice de correction invA — ce qui permet
d'obtenir un cap précis dans toutes les directions.
C'est la première étape de la calibration, avant create_magNorthWest.py qui oriente le référentiel vers
le Nord.

Usage :
  python3 create_donnees_mag.py
"""

import sys
import time
import numpy as np

sys.path.append('/home/ue32/drivers-ddboat-v2')
from imu9_driver_v2 import Imu9IO

DURATION   = 30    # secondes d'enregistrement
SAMPLE_DT  = 0.05  # intervalle entre deux mesures (20 Hz)

if __name__ == "__main__":
    imu = Imu9IO()

    print("=" * 55)
    print("  Création de donnees_mag.csv — Calibration magnéto")
    print("=" * 55)
    print("""
  - Reste loin de tout objet métallique
  - 30 sec d'enregistrement
""")
    input("→ Appuie sur ENTRÉE pour démarrer l'enregistrement...")

    samples = []
    n_total = int(DURATION / SAMPLE_DT)

    print("\n  Enregistrement en cours... Fais tourner le bateau !\n")

    for i in range(n_total):
        x, y, z = imu.read_mag_raw()
        samples.append([x, y, z])

        # Barre de progression
        pct = int((i + 1) / n_total * 40)
        bar = "#" * pct + "-" * (40 - pct)
        remaining = int((n_total - i - 1) * SAMPLE_DT)
        print("\r  [{}] {}s restantes  x={:6.0f} y={:6.0f} z={:6.0f}".format(
            bar, remaining, x, y, z), end="")
        sys.stdout.flush()
        time.sleep(SAMPLE_DT)

    print("\n\n  Enregistrement terminé ! {} mesures collectées.".format(len(samples)))

    # Sauvegarde au format attendu par cap_sender_v3.py (3 colonnes : x1, x2, x3)
    data = np.array(samples)
    np.savetxt('donnees_mag.csv', data, delimiter=',', fmt='%.6f')

    print("[OK] donnees_mag.csv sauvegardé.")
