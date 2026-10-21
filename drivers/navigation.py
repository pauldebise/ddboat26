# -*- coding: utf-8 -*-
"""Briques de navigation : gradient de ligne et gradient ponctuel.

Tout est exprime dans le repere local du lac (x = est, y = nord quand la
rotation est nulle) ; le module n'utilise que numpy et se teste sans
materiel.

Un chemin est une suite de sommets [P0, P1, ... Pn]. Le bateau suit les
segments P0->P1, P1->P2, ... Sur le segment A->B, le vecteur de guidage est

    direction = k_ligne * (-q)  +  k_point * unitaire(B - position)

- gradient de ligne : -q est la perpendiculaire qui ramene le bateau sur le
  segment [A, B] (q = vecteur du point le plus proche du segment vers le
  bateau). k_ligne est en 1/m : a 20 m du segment, avec k_ligne = 0,05, la
  composante laterale egale la composante vers B. Ce gain doit rester
  faible : le cap du bateau repond avec ~3 s de constante de temps et 0,5 s
  de retard, et la boucle de ligne doit etre bien plus lente que celle du cap
  (a 0,2, essais du 21/10 : oscillation de +-60 deg et +-3 m, periode 20 s) ;
- gradient ponctuel : le vecteur unitaire qui pointe vers le sommet vise.

Avec k_ligne = 0, il ne reste que le gradient ponctuel (ralliement d'un
point). Comme maintien_cap() n'utilise que la direction du vecteur, seul le
rapport k_ligne / k_point compte.

Le sommet B est atteint quand le bateau passe a moins de rayon_arrivee de B,
ou qu'il depasse B (projection au-dela du segment) a moins de 2,5 rayons.
"""

import math

import numpy as np


class SuiveurChemin(object):
    K_LIGNE = 0.05
    K_POINT = 1.0

    def __init__(self, points, rayon_arrivee=3.0, k_ligne=None, k_point=None,
                 rayon_final=None):
        self.points = np.array(points, dtype=float).reshape(-1, 2)
        if len(self.points) < 2:
            raise ValueError("un chemin demande au moins deux points")
        self.rayon_arrivee = float(rayon_arrivee)
        # Rayon du dernier sommet (par defaut le meme) : plus grand, il arrete
        # le bateau avant un ponton.
        self.rayon_final = self.rayon_arrivee if rayon_final is None else float(rayon_final)
        self.k_ligne = self.K_LIGNE if k_ligne is None else float(k_ligne)
        self.k_point = self.K_POINT if k_point is None else float(k_point)
        self.segment = 0          # indice du segment courant : points[i] -> points[i+1]
        self.termine = False

    @property
    def nombre_segments(self):
        return len(self.points) - 1

    def segment_courant(self):
        i = min(self.segment, self.nombre_segments - 1)
        return self.points[i], self.points[i + 1]

    def longueur_totale(self):
        return float(np.sum(np.linalg.norm(np.diff(self.points, axis=0), axis=1)))

    def ecart_au_chemin(self, position):
        """Distance minimale de `position` a la ligne brisee complete."""
        p = np.asarray(position, dtype=float)
        meilleur = float("inf")
        for i in range(self.nombre_segments):
            a, b = self.points[i], self.points[i + 1]
            ab = b - a
            carre = float(np.dot(ab, ab))
            t = 0.0 if carre < 1e-12 else max(0.0, min(1.0, float(np.dot(p - a, ab)) / carre))
            meilleur = min(meilleur, float(np.linalg.norm(p - (a + t * ab))))
        return meilleur

    def rayon_courant(self):
        """Rayon d'arrivee du sommet vise."""
        if self.segment + 1 >= self.nombre_segments:
            return self.rayon_final
        return self.rayon_arrivee

    def sur_dernier_segment(self):
        return self.segment + 1 >= self.nombre_segments

    def _geometrie(self, position):
        a, b = self.segment_courant()
        ab = b - a
        longueur = float(np.linalg.norm(ab))
        if longueur < 0.5:
            # Segment degenere : seul le gradient ponctuel a un sens.
            u = np.array([0.0, 0.0])
            s = 0.0
            q = np.array([0.0, 0.0])
            lateral = 0.0
        else:
            u = ab / longueur
            w = position - a
            s = float(np.dot(w, u))
            q = w - max(0.0, min(longueur, s)) * u
            # Positif a droite du sens de marche.
            lateral = float(u[1] * w[0] - u[0] * w[1])
        vers_b = b - position
        dist_b = float(np.linalg.norm(vers_b))
        return a, b, u, longueur, s, q, lateral, vers_b, dist_b

    def evaluer(self, position):
        """Met a jour le chemin et retourne le guidage pour `position` (x, y).

        Le dict retourne contient direction (vecteur), segment, a, b, s,
        longueur, lateral (m, positif a droite), dist_cible (m),
        sommet_atteint (indice du sommet franchi ou None) et termine.
        """
        position = np.asarray(position, dtype=float)
        sommet_atteint = None
        while not self.termine:
            a, b, u, longueur, s, q, lateral, vers_b, dist_b = self._geometrie(position)
            rayon = self.rayon_courant()
            atteint = dist_b <= rayon or (
                longueur >= 0.5 and s >= longueur
                and abs(lateral) <= 2.5 * rayon)
            if not atteint:
                break
            sommet_atteint = self.segment + 1
            if self.segment + 1 >= self.nombre_segments:
                self.termine = True
            else:
                self.segment += 1
        a, b, u, longueur, s, q, lateral, vers_b, dist_b = self._geometrie(position)
        if dist_b > 1e-6:
            vecteur = self.k_point * vers_b / dist_b
        else:
            vecteur = np.zeros(2)
        vecteur = vecteur - self.k_ligne * q
        return {
            "direction": vecteur,
            "segment": self.segment,
            "a": a,
            "b": b,
            "s": s,
            "longueur": longueur,
            "lateral": lateral,
            "dist_cible": dist_b,
            "sommet_atteint": sommet_atteint,
            "termine": self.termine,
        }


def construire_chemin_ferme(depart, sommets, tours=1):
    """Chemin : depart, puis les sommets, retour au premier, `tours` fois.

    Le premier segment va du depart au premier sommet ; chaque tour suivant
    repart du premier sommet. Les `tours` tours se terminent sur le premier
    sommet.
    """
    chemin = [list(depart), list(sommets[0])]
    for _ in range(max(1, int(tours))):
        for sommet in sommets[1:]:
            chemin.append(list(sommet))
        chemin.append(list(sommets[0]))
    return chemin


def point_a_distance(origine, cap_deg, distance):
    """Point a `distance` m de `origine`, dans le cap boussole `cap_deg`."""
    cap = math.radians(cap_deg)
    return [float(origine[0]) + distance * math.sin(cap),
            float(origine[1]) + distance * math.cos(cap)]
