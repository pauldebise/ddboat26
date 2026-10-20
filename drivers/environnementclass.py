from ddboatclass import DDBoat
import numpy as np

coord_couloir = np.array([[0.0, 0.0], [1.0, 0.0]])
coord_bordure_gauche = np.array([[-1.0, 0.0], [-1.0, 1.0]])
coord_bordure_droite = np.array([[1.0, 0.0], [1.0, 1.0]])


class Environnement:
    def __init__(self, ddboat: DDBoat):
        self.ddboat = ddboat
        self.home = np.array([0.0, 0.0])  # position de la base dans le repere local (x, y).

    def gradient_artificiel_point(self, x_pt: float, y_pt: float, x_pos: float, y_pos: float, attract: bool):
        signe = (-1) if attract else 1
        return signe * np.array([x_pos-x_pt, y_pos-y_pt])

    def gradient_artificiel_ligne(self, x_n: float, y_n: float, x_pt: float, y_pt: float, x_pos: float, y_pos: float):
        n = np.array([x_n, y_n])
        Q = n@n.T
        pp = np.array([x_pos-x_pt, y_pos-x_pt])
        return -2 * Q@pp

    def gradient_artificiel_uniforme(self, x_v, y_v):
        return np.array([x_v, y_v])

    