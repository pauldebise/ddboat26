# -*- coding: utf-8 -*-
"""
cap_sender_v3 — A lancer sur le DD-Boat
------------------------------------------------
Ce script tourne sur le DD-Boat et fait deux choses :
1. Calculer le cap à partir du magnétomètre :

Il charge la calibration (invA.csv et xbar.csv), ou la recalcule depuis donnees_mag.csv et magNorthWest.csv si elle n'existe pas encore
Il lit le magnétomètre en continu et calcule le cap en radians via compute_heading

2. Envoyer le cap au PC via UDP :

Il envoie à chaque itération (20 Hz) un message "heading_rad heading_deg" à l'IP du PC que tu passes en argument
C'est ce que cap_display.py reçoit pour afficher la boussole
Si tu exécute le script sur le ddBOAT, il affiche une boussole sur l'ordi.
Usage :
  python3 cap_sender_v3.py
"""
from __future__ import print_function, division
import sys, os, time
import numpy as np
import socket

# ── Chemin vers les drivers DD-Boat ─────────────────────────────────────────
sys.path.append('/home/ue32/drivers-ddboat-v2')
from imu9_driver_v2 import Imu9IO

# ── Arguments ────────────────────────────────────────────────────────────────
if len(sys.argv) < 2:
    print("Usage : python cap_sender.py <IP_du_PC> [port]")
    sys.exit(1)

PC_IP = sys.argv[1]
PC_PORT = int(sys.argv[2]) if len(sys.argv) >= 3 else 5005


# ── Fonctions d'algèbre linéaire alternatives à SciPy ────────────────────────
def my_sqrtm(M):
    """
    Alternative à scipy.linalg.sqrtm pour matrices symétriques définies positives.
    Utilise la décomposition en valeurs propres (Eigendecomposition).
    """
    evals, evecs = np.linalg.eigh(M)
    # On s'assure que les valeurs propres ne sont pas négatives (bruit numérique)
    evals = np.maximum(evals, 0)
    return evecs.dot(np.diag(np.sqrt(evals))).dot(evecs.T)


# ── Chargement de la calibration ─────────────────────────────────────────────
def load_calibration():
    try:
        invA = np.loadtxt("invA.csv", delimiter=",")
        xbar = np.loadtxt("xbar.csv", delimiter=",")
        print("[OK] Calibration chargee depuis invA.csv et xbar.csv")
        return invA, xbar
    except IOError:
        print("[WARN] Fichiers de calibration introuvables. Calcul manuel...")
        return compute_calibration()


def compute_calibration():
    # Chargement des données brutes
    x1, x2, x3 = np.loadtxt('donnees_mag.csv', delimiter=',', unpack=True)
    data = np.loadtxt('magNorthWest.csv')
    xnorth = data[:3].reshape(3, 1)
    xwest = data[3:].reshape(3, 1)

    # Fit ellipsoïde (Utilisation de np.linalg.inv)
    X = np.column_stack([x1 ** 2, x2 ** 2, x3 ** 2, x1 * x2, x1 * x3, x2 * x3, x1, x2, x3])
    p = np.linalg.inv(X.T.dot(X)).dot(X.T).dot(np.ones((len(x1), 1)))

    P = np.array([[p[0, 0], 1 / 2 * p[3, 0], 1 / 2 * p[4, 0]],
                  [1 / 2 * p[3, 0], p[1, 0], 1 / 2 * p[5, 0]],
                  [1 / 2 * p[4, 0], 1 / 2 * p[5, 0], p[2, 0]]])
    c = np.array([p[6], p[7], p[8]])

    xbar = -1 / 2 * np.linalg.inv(P).dot(c)
    Q = P / (1 + xbar.T.dot(P).dot(xbar))

    # Procédure de pointage
    I = 64 * 3.14 / 180
    ynorth = np.array([[np.cos(I)], [0], [-np.sin(I)]])
    ywest = np.array([[0], [-np.cos(I)], [-np.sin(I)]])
    yup = np.cross(ynorth.flatten(), ywest.flatten()).reshape(3, 1)
    Y = np.hstack([ynorth, ywest, yup])

    # Utilisation de my_sqrtm à la place de scipy.linalg.sqrtm
    sqrtQ = my_sqrtm(Q)
    vnorth = sqrtQ.dot(xnorth - xbar)
    vwest = sqrtQ.dot(xwest - xbar)
    vup = np.cross(vnorth.flatten(), vwest.flatten()).reshape(3, 1)
    V = np.hstack([vnorth, vwest, vup])

    R = Y.dot(np.linalg.inv(V))
    invA = R.dot(sqrtQ)

    # Sauvegarde
    np.savetxt("invA.csv", invA, delimiter=",")
    np.savetxt("xbar.csv", xbar.flatten(), delimiter=",")
    print("[OK] Calibration calculee et sauvegardee.")
    return invA, xbar.flatten()


# ── Calcul du cap ─────────────────────────────────────────────────────────────
def compute_heading(raw_mag, invA, xbar):
    x = np.array(raw_mag).reshape(3, 1)
    xb = xbar.reshape(3, 1)
    yhat = invA.dot(x - xb)
    heading = np.arctan2(float(yhat[1]), float(yhat[0])) % (2 * np.pi)
    return heading


# ── Programme principal ───────────────────────────────────────────────────────
if __name__ == "__main__":
    invA, xbar = load_calibration()
    imu = Imu9IO()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("\n[INFO] Envoi du cap (SANS SCIPY) vers {}:{}".format(PC_IP, PC_PORT))

    dt = 0.05
    try:
        while True:
            xmag, ymag, zmag = imu.read_mag_raw()
            heading_rad = compute_heading([xmag, ymag, zmag], invA, xbar)
            heading_deg = np.degrees(heading_rad)
            heading_deg = heading_deg % 360
            heading_rad = np.radians(heading_deg)

            msg = "{:.4f} {:.1f}\n".format(heading_rad, heading_deg)
            sock.sendto(msg.encode(), (PC_IP, PC_PORT))

            print("\r  Cap = {:+7.1f} deg  ".format(heading_deg), end="")
            sys.stdout.flush()
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[INFO] Arret.")
        sock.close()