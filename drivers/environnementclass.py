from ddboatclass import DDBoat
import numpy as np


rotation = 0.0
coord_couloir = np.array([[48.198863,-3.015070], [48.199072,-3.016258]])
coord_bordure_gauche = np.array([[48.197408,-3.01506], [48.1970689,-3.017328]])
coord_bordure_droite = np.array([[48.199707,-3.016404], [48.199814,-3.015137]])
coord_home = np.array([48.198863,-3.015070])

ddb_calcul = DDBoat(None, None, None)  # Instance de DDBoat pour les calculs de coordonnées
coord_bordure_gauche[0] = ddb_calcul.convert_cart_position(coord_bordure_gauche[0][0], coord_bordure_gauche[0][1], coord_home, rotation)
coord_bordure_gauche[1] = ddb_calcul.convert_cart_position(coord_bordure_gauche[1][0], coord_bordure_gauche[1][1], coord_home, rotation)
coord_bordure_droite[0] = ddb_calcul.convert_cart_position(coord_bordure_droite[0][0], coord_bordure_droite[0][1], coord_home, rotation)
coord_bordure_droite[1] = ddb_calcul.convert_cart_position(coord_bordure_droite[1][0], coord_bordure_droite[1][1], coord_home, rotation)
coord_couloir[0] = ddb_calcul.convert_cart_position(coord_couloir[0][0], coord_couloir[0][1], coord_home, rotation)
coord_couloir[1] = ddb_calcul.convert_cart_position(coord_couloir[1][0], coord_couloir[1][1], coord_home, rotation)
print("Coordonnées converties.")



class Environnement:

    def __init__(self):
        self.home = np.array([0.0, 0.0])  # position de la base dans le repere local (x, y).
        self.home_latlong = coord_home  # position de la base en latitude et longitude.
        self.rotation = rotation  # angle de rotation du repere local par rapport au repere global (en radians).
        self.couloir = coord_couloir  # position du couloir dans le repere local (x, y).
        self.bordure_gauche = coord_bordure_gauche  # position de la bordure gauche dans le repere local (x, y).
        self.bordure_droite = coord_bordure_droite  # position de la bordure droite dans le repere local (x, y).

    def gradient_artificiel_point(self, x_pt: float, y_pt: float, x_pos: float, y_pos: float, attract: bool):
        signe = (-1) if attract else 1
        return signe * np.array([x_pos-x_pt, y_pos-y_pt])

    def gradient_artificiel_ligne(self, x_n: float, y_n: float, x_pt: float, y_pt: float, x_pos: float, y_pos: float):
        n = np.array([x_n, y_n])
        Q = np.outer(n, n)
        pp = np.array([x_pos-x_pt, y_pos-y_pt])
        return -2 * (Q @ pp)

    def gradient_artificiel_uniforme(self, x_v, y_v):
        return np.array([x_v, y_v])

    def direction(self, x, y):
        gradient_ligne = self.gradient_artificiel_ligne(self.couloir[1][0]-self.couloir[0][0], self.couloir[1][1]-self.couloir[0][1], self.couloir[0][0], self.couloir[0][1], x, y)
        v_dir = coord_couloir[0] - coord_couloir[1]
        v_dir = v_dir / np.linalg.norm(v_dir)  # Normalisation du vecteur directeur
        gradient_uniforme = self.gradient_artificiel_uniforme(v_dir[0], v_dir[1])
        return gradient_ligne + gradient_uniforme

    def lim(self, x, y):
        """Vérifie si le point (x, y) est à l'intérieur des limites du couloir."""
        # Vérification des bordures gauche et droite
        if (coord_bordure_gauche[0][0] <= x <= coord_bordure_droite[0][0]) and (coord_bordure_gauche[1][1] <= y <= coord_bordure_droite[1][1]):
            return True
        return False
