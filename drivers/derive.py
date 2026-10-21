# -*- coding: utf-8 -*-
"""Estimation de la derive du bateau (vent, courant) a partir du GPS.

Principe
--------
Entre deux fix GPS, la vitesse sol mesuree est la somme de deux termes :

    v_sol = s * (sin cap, cos cap) + w

- s : vitesse du bateau dans l'eau, le long de son cap (proportionnelle a la
  commande moteur) ;
- w : derive (vent + courant), vecteur (est, nord) en m/s.

Sur une fenetre glissante de quelques dizaines de secondes, une regression
lineaire donne s et w. Contrairement a un simple ecart « route GPS - cap »,
le vecteur w reste valable quand le cap change (virage d'un polygone) :
l'angle de crabe se recalcule pour chaque nouvelle route.

Pour suivre une route sol d (cap boussole), le bateau doit pointer vers

    cap = d + asin(-(w . droite(d)) / s)

ou droite(d) = (cos d, -sin d) est la perpendiculaire a droite de la route.
Le biais du compas se retrouve en partie dans w tant que le cap reste a peu
pres constant : la correction vaut donc aussi pour une calibration imparfaite.

Le module n'utilise que numpy : il se teste sans materiel.
"""

import math
import threading
from collections import deque

import numpy as np


class EstimateurDerive(object):
    VITESSE_NOMINALE = 1.3        # vitesse dans l'eau a la commande de reference (m/s)
    COMMANDE_REFERENCE = 150.0    # commande moteur moyenne correspondante
    FENETRE = 25.0                # age maximal des mesures utilisees (s)
    DEMI_VIE = 8.0                # les mesures recentes comptent davantage (s)
    VALIDITE = 60.0               # duree pendant laquelle on garde une estimation sans mesure (s)
    MIN_ECHANTILLONS = 5
    DISPERSION_MAX = math.radians(10.0)   # cap quasi constant sur l'intervalle
    COMMANDE_MIN = 80.0           # bateau propulse (sinon la mesure ne dit rien)
    POIDS_VITESSE = 1.0           # force de l'a priori sur s
    POIDS_DERIVE = 0.3            # force de l'a priori w = 0
    SEUIL_ABERRANT = 0.35         # m/s : au-dela, une mesure compte de moins en moins
    CRAB_MAX = math.radians(50.0)

    def __init__(self):
        # Lu par le serveur de l'IHM, le thread GPS et la mission.
        self._verrou = threading.Lock()
        self.echantillons = deque()
        self.derniere = None
        self.instant_derniere = None
        self.acceptes = 0
        self.rejetes = 0

    def reinitialiser(self):
        with self._verrou:
            self.echantillons.clear()
            self.derniere = None
            self.instant_derniere = None

    def ajouter(self, instant, duree, vitesse_est, vitesse_nord, cap,
                dispersion_cap, commande):
        """Ajoute la mesure d'un intervalle entre deux fix. Retourne True si admise."""
        if (commande < self.COMMANDE_MIN
                or dispersion_cap > self.DISPERSION_MAX
                or not (0.4 <= duree <= 3.0)):
            self.rejetes += 1
            return False
        with self._verrou:
            self.echantillons.append((instant, vitesse_est, vitesse_nord, cap, commande))
            self.acceptes += 1
            while self.echantillons and instant - self.echantillons[0][0] > self.FENETRE:
                self.echantillons.popleft()
        return True

    def estimer(self, instant):
        """Retourne un dict (s, est, nord, n, rms, age) ou None."""
        with self._verrou:
            recents = [e for e in self.echantillons if instant - e[0] <= self.FENETRE]
            derniere = self.derniere
            instant_derniere = self.instant_derniere
        if len(recents) >= self.MIN_ECHANTILLONS:
            lignes = []
            mesures = []
            poids = []
            for (t, ve, vn, cap, cmd) in recents:
                sigma = cmd / self.COMMANDE_REFERENCE
                p = math.sqrt(0.5 ** ((instant - t) / self.DEMI_VIE))
                lignes.append([sigma * math.sin(cap), 1.0, 0.0])
                mesures.append(ve)
                poids.append(p)
                lignes.append([sigma * math.cos(cap), 0.0, 1.0])
                mesures.append(vn)
                poids.append(p)
            a = np.array(lignes)
            b = np.array(mesures)
            w = np.array(poids)
            ls = math.sqrt(self.POIDS_VITESSE)
            lw = math.sqrt(self.POIDS_DERIVE)
            a_ridge = np.array([[ls, 0.0, 0.0], [0.0, lw, 0.0], [0.0, 0.0, lw]])
            b_ridge = np.array([ls * self.VITESSE_NOMINALE, 0.0, 0.0])
            a_tot = np.vstack([a * w[:, None], a_ridge])
            b_tot = np.concatenate([b * w, b_ridge])
            # Regression robuste (Huber, quelques iterations) : une mesure
            # aberrante (bateau pris a la main, choc, rafale) est ecartee au
            # lieu de fausser l'estimation pendant 25 s.
            poids_robuste = np.ones(len(a))
            for _ in range(4):
                pr = np.concatenate([poids_robuste, np.ones(3)])
                a_p = a_tot * pr[:, None]
                solution = np.linalg.solve(a_tot.T.dot(a_p), a_p.T.dot(b_tot))
                residu = a.dot(solution) - b
                # Norme du residu de chaque mesure (deux lignes : est, nord).
                norme = np.hypot(residu[0::2], residu[1::2])
                poids_mesure = np.minimum(1.0, self.SEUIL_ABERRANT / np.maximum(norme, 1e-9))
                poids_robuste = np.repeat(poids_mesure, 2)
            derniere = {
                "s": float(min(3.0, max(0.3, solution[0]))),
                "est": float(solution[1]),
                "nord": float(solution[2]),
                "n": len(recents),
                "rms": float(math.sqrt(np.mean(residu ** 2))),
            }
            instant_derniere = instant
            with self._verrou:
                self.derniere = derniere
                self.instant_derniere = instant_derniere
        if derniere is None:
            return None
        age = instant - instant_derniere
        if age > self.VALIDITE:
            return None
        resultat = dict(derniere)
        resultat["age"] = age
        return resultat

    def cap_pour_route(self, route, instant):
        """Cap a prendre pour que la route sol soit `route` (rad, boussole).

        Retourne (cap, crabe, estimation) ; crabe = cap - route (rad). Sans
        estimation exploitable, le cap vaut la route.
        """
        estimation = self.estimer(instant)
        if estimation is None:
            return route, 0.0, None
        droite_est = math.cos(route)
        droite_nord = -math.sin(route)
        derive_laterale = estimation["est"] * droite_est + estimation["nord"] * droite_nord
        rapport = -derive_laterale / estimation["s"]
        rapport = max(-math.sin(self.CRAB_MAX), min(math.sin(self.CRAB_MAX), rapport))
        crabe = math.asin(rapport)
        return (route + crabe) % (2 * math.pi), crabe, estimation
