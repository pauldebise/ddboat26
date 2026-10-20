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
        """Attire perpendiculairement vers le segment de couloir C1-C2.

        (x_n, y_n) est le vecteur C2 - C1, et (x_pt, y_pt) le point C1.

        L'ancien calcul utilisait n n^T : il projetait le vecteur sur la
        direction du couloir, donc il ne restait aucune composante
        perpendiculaire et le bateau ne pouvait pas rejoindre la ligne.
        Ici on calcule le vecteur q qui va du point le plus proche du
        segment vers le bateau, puis on renvoie -q : c'est l'attraction
        perpendiculaire vers le couloir.

        La projection est bornee entre 0 et |n| pour viser le segment
        C1-C2, et non la droite infinie qui le prolonge.
        """
        n = np.array([x_n, y_n], dtype=float)
        longueur = np.linalg.norm(n)
        if longueur < 1e-9:
            return np.zeros(2)

        u = n / longueur
        p = np.array([x_pos - x_pt, y_pos - y_pt], dtype=float)

        # Coordonnee le long du couloir, bornee au segment C1-C2.
        s = float(np.dot(p, u))
        s = max(0.0, min(longueur, s))

        # q va du segment vers le bateau ; -q attire donc vers le segment.
        q = p - s * u
        return -q

    def gradient_artificiel_uniforme(self, x_v, y_v):
        return np.array([x_v, y_v])

    def direction(self, x, y):
        # Gradient de ligne : attraction perpendiculaire vers le segment
        # C1-C2. Il s'annule sur le couloir et augmente avec l'ecart lateral.
        gradient_ligne = self.gradient_artificiel_ligne(
            self.couloir[1][0] - self.couloir[0][0],
            self.couloir[1][1] - self.couloir[0][1],
            self.couloir[0][0],
            self.couloir[0][1],
            x, y)

        # Gradient uniforme : direction globale du couloir vers la base.
        # C1 est la base (couloir[0]), C2 l'entree (couloir[1]).
        c1 = self.couloir[0]
        c2 = self.couloir[1]
        n = c2 - c1
        longueur = np.linalg.norm(n)
        if longueur < 1e-9:
            return gradient_ligne

        u = n / longueur
        v_dir = c1 - c2
        norme = np.linalg.norm(v_dir)
        if norme < 1e-9:
            return gradient_ligne
        v_dir = v_dir / norme

        # Le terme uniforme ne doit pas pousser le bateau au-dela de la
        # base : sinon, derriere C1, il s'oppose a l'attraction vers la
        # base et cree un equilibre artificiel au-dela de la base.
        # On l'annule derriere C1 et on le fait decroitre lineairement
        # dans les 10 derniers metres pour garder une transition douce.
        s = float(np.dot(np.array([x, y], dtype=float) - c1, u))
        if s <= 0.0:
            facteur_uniforme = 0.0
        elif s < 10.0:
            facteur_uniforme = s / 10.0
        else:
            facteur_uniforme = 1.0

        gradient_uniforme = self.gradient_artificiel_uniforme(
            facteur_uniforme * v_dir[0],
            facteur_uniforme * v_dir[1])

        # Ponderation des deux composantes. Seul le rapport k_ligne /
        # k_uniforme compte pour le cap, puisque maintien_cap() utilise
        # atan2() sur le vecteur resultat.
        # A 50 m d'ecart lateral, k_ligne * 50 vaut 4 face a
        # k_uniforme = 1 : l'attraction perpendiculaire domine et le
        # bateau rejoint le couloir avant de suivre la composante
        # longitudinale. Les deux termes s'equilibrent vers 12 m.
        # Ces gains sont a ajuster selon le rayon de courbure du bateau.
        k_ligne = 0.08
        k_uniforme = 1.0
        return k_ligne * gradient_ligne + k_uniforme * gradient_uniforme

    def lim(self, x, y):
        """Indique si (x, y) est entre les bordures laterales du couloir.

        Il n'y a pas de bordure avant ou arriere : on ne teste que les deux
        demi-plans definis par les droites de bordure. Un point situe au-dela
        de C1 ou de C2 mais toujours entre les deux droites reste donc dans
        les limites.

        L'ancien test comparait x et y a un rectangle, avec des bornes
        inversees (0.74 <= x <= -98.87), donc il renvoyait toujours False.
        Ici on calcule une distance signee a chaque bordure :
          - d_droite > 0 : point a droite de la bordure droite ;
          - d_gauche > 0 : point a gauche de la bordure gauche.
        Le point est dans les limites si les deux distances sont <= 0.

        x et y doivent etre exprimes dans le meme repere local que
        self.bordure_gauche et self.bordure_droite.
        """
        point = np.array([x, y], dtype=float)

        # Direction du couloir, de la base C1 vers l'entree C2.
        c1 = self.couloir[0]
        c2 = self.couloir[1]
        u = np.array(c2 - c1, dtype=float)
        norme_u = np.linalg.norm(u)
        if norme_u < 1e-9:
            return False
        u = u / norme_u

        # Normales exterieures attendues, dans un repere x=est, y=nord.
        # (u_y, -u_x) est le cote droit du couloir ; son oppose est le gauche.
        normale_droite = np.array([u[1], -u[0]])
        normale_gauche = -normale_droite

        def distance_signee(bordure, normale_attendue):
            """Distance signee a la droite support de la bordure.

            La normale de la bordure est orientee dans le sens de
            normale_attendue, pour que d > 0 signifie "exterieur".
            """
            a = np.array(bordure[0], dtype=float)
            b = np.array(bordure[1], dtype=float)
            v = b - a
            norme_v = np.linalg.norm(v)
            if norme_v < 1e-9:
                return None
            v = v / norme_v

            # Normale de la droite ; on la retourne si elle pointe vers
            # l'interieur au lieu de l'exterieur.
            normale = np.array([-v[1], v[0]])
            if np.dot(normale, normale_attendue) < 0.0:
                normale = -normale

            return float(np.dot(point - a, normale))

        d_droite = distance_signee(self.bordure_droite, normale_droite)
        d_gauche = distance_signee(self.bordure_gauche, normale_gauche)

        # Une bordure degeneree ne permet pas de conclure : par prudence,
        # on considere le point hors limites.
        if d_droite is None or d_gauche is None:
            return False

        # Aucune condition sur la coordonnee le long du couloir : il n'y a
        # pas de bordure avant/arriere.
        return d_droite <= 0.0 and d_gauche <= 0.0
