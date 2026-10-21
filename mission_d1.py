# -*- coding: utf-8 -*-
"""Mission D1 : aller vers D1, sortie de zone, tour complet, retour au couloir.

Lancee depuis l'IHM (bouton Mission ou `python3 ihm_lac.py --mission d1`).

Deroulement
-----------
1. ATTENTE_GPS : attente d'un fix GPS (bateau pose a la base).
2. DEPART      : compte a rebours (attente), le cap est enregistre.
3. ALLER_D1    : navigation vers D1 (premier point de la bordure droite).
                 A moins de rayon_d1 metres de D1, le bateau garde le cap
                 d'approche pour franchir la bordure.
4. HORS_LIMITES: sortie confirmee sur `confirmations` fix GPS successifs.
5. TOUR        : tour complet de 360 degres.
6. RETOUR      : retour a la base par le couloir (Environnement.direction()).
7. ARRIVEE     : moteurs coupes a moins de rayon_arrivee metres de la base.

Le suivi de cap utilise le regulateur PID et la compensation de derive de
DDBoat (voir drivers/ddboatclass.py).
"""

import math
import time

import numpy as np

from ddboatclass import angle_diff, dist_cart
from session import Mission


class MissionD1(Mission):
    NOM = "d1"
    TITRE = "Mission D1 : aller, sortie, tour complet, retour"
    DESCRIPTION = ("Part de la base vers D1, sort de la zone, fait un tour "
                   "complet puis revient a la base par le couloir.")
    PARAMETRES = [
        {"nom": "attente", "label": "Compte a rebours (s)", "defaut": 10.0},
        {"nom": "confirmations", "label": "Fix hors limites pour valider",
         "defaut": 3, "type": "entier"},
        {"nom": "rayon_arrivee", "label": "Rayon d'arrivee a la base (m)", "defaut": 5.0},
        {"nom": "rayon_d1", "label": "Cap d'approche garde a moins de (m)", "defaut": 10.0},
        {"nom": "depassement_max", "label": "Depassement max apres D1 (m)", "defaut": 20.0},
        {"nom": "distance_max", "label": "Distance max a la base (m)", "defaut": 250.0},
        {"nom": "timeout_aller", "label": "Duree max de l'aller (s)", "defaut": 400.0},
        {"nom": "timeout_retour", "label": "Duree max du retour (s)", "defaut": 900.0},
        {"nom": "timeout_gps", "label": "Attente max du GPS (s)", "defaut": 120.0},
    ]

    @classmethod
    def apercu(cls, session, parametres):
        env = session.env
        return [[0.0, 0.0], [float(env.bordure_droite[0][0]), float(env.bordure_droite[0][1])]]

    def _aller_d1(self, depart):
        """Retourne "sortie", ou la raison de l'abandon."""
        s, ddb, env, opts = self.s, self.ddb, self.env, self.p
        d1 = self.d1
        debut = time.monotonic()
        hors_consecutifs = 0
        dernier_fix = ddb.compteur_fix_gps
        direction_finale = None  # cap d'approche garde pres de D1
        distance_finale = None   # distance a D1 au moment du passage
        gps_perime = False
        precedent = time.monotonic()
        ddb.reinitialiser_asservissement()
        s.definir_guidage(chemin=[[float(depart[0]), float(depart[1])],
                                  [float(d1[0]), float(d1[1])]],
                          b=[float(d1[0]), float(d1[1])])
        try:
            while True:
                s.verifier_arret()
                maintenant = time.monotonic()
                dt = maintenant - precedent
                precedent = maintenant
                if maintenant - debut > opts["timeout_aller"]:
                    return "timeout"

                position, age = s.position_fraiche()
                if position is None:
                    ddb._arreter_moteurs()
                    if not gps_perime:
                        s.ecrire_journal("gps_perime", {"age": age})
                        s.definir_message("GPS perdu, moteurs coupes")
                        gps_perime = True
                    time.sleep(0.2)
                    continue
                if gps_perime:
                    s.ecrire_journal("gps_retabli", {"age": age})
                    gps_perime = False

                x, y = position
                dist_base = dist_cart(position, env.home)
                vers_d1 = d1 - position
                dist_d1 = float(np.linalg.norm(vers_d1))
                if dist_base > opts["distance_max"]:
                    return "distance_max"

                # Pres de D1, la direction vers D1 devient instable (bruit
                # GPS) : on garde la direction d'approche jusqu'a la sortie.
                if direction_finale is None and dist_d1 <= opts["rayon_d1"]:
                    direction_finale = d1 - depart
                    direction_finale /= np.linalg.norm(direction_finale)
                    distance_finale = dist_d1
                    s.ecrire_journal("d1_atteint", {
                        "x": x, "y": y, "dist_d1": dist_d1})
                    s.definir_message("D1 atteint, franchissement")

                if direction_finale is not None:
                    direction = direction_finale
                    # Distance parcourue au-dela de D1 le long de l'approche.
                    depassement = float(np.dot(position - d1, direction_finale))
                    if depassement > opts["depassement_max"]:
                        return "sortie_non_detectee"
                else:
                    direction = vers_d1 / max(dist_d1, 1e-6)
                    depassement = None

                # Detection de sortie : uniquement sur de nouveaux fix.
                dans_zone = bool(env.lim(x, y))
                if ddb.compteur_fix_gps != dernier_fix:
                    dernier_fix = ddb.compteur_fix_gps
                    if dans_zone:
                        hors_consecutifs = 0
                    else:
                        hors_consecutifs += 1
                        s.ecrire_journal("hors_limites_fix", {
                            "x": x, "y": y, "compte": hors_consecutifs})
                        s.definir_message("hors limites {}/{}".format(
                            hors_consecutifs, opts["confirmations"]))
                        if hors_consecutifs >= opts["confirmations"]:
                            return "sortie"

                cible, cap, gauche, droite = ddb.maintien_cap(direction, env.rotation)
                sol = ddb.vitesse_sol or {}
                s.ecrire_journal("nav", dict({
                    "mission": "d1",
                    "x": x, "y": y, "age": age, "dt": dt,
                    "dist_base": dist_base, "dist_d1": dist_d1,
                    "depassement": depassement, "dans_zone": dans_zone,
                    "direction": direction, "cap_cible": math.degrees(cible),
                    "cap": math.degrees(cap),
                    "erreur": math.degrees(angle_diff(cible, cap)),
                    "moteurs": (gauche, droite), "mag": ddb.dernier_mag_brut,
                    "v_sol": sol.get("vitesse"), "cog": sol.get("cog"),
                }, **ddb.dernier_asservissement))
                s.definir_guidage(dist_cible=dist_d1,
                                  direction=[float(direction[0]), float(direction[1])],
                                  route=ddb.dernier_asservissement.get("route"),
                                  crabe=ddb.dernier_asservissement.get("crabe"))
                if direction_finale is None:
                    s.definir_message("D1 a {:.0f} m".format(dist_d1))
                time.sleep(0.05)
        finally:
            ddb._arreter_moteurs()
            s.definir_guidage()
            if distance_finale is not None:
                s.ecrire_journal("aller_fin", {"dist_d1_passage": distance_finale})

    def executer(self):
        s, ddb, env, opts = self.s, self.ddb, self.env, self.p
        self.d1 = np.array(env.bordure_droite[0], dtype=float)

        s.definir_phase("ATTENTE_GPS", "en attente d'un fix GPS")
        depart = s.attendre_gps(opts["timeout_gps"])
        if depart is None:
            s.definir_phase("ECHEC", "aucun fix GPS")
            return 1, "pas_de_gps"

        distance_base = dist_cart(depart, env.home)
        s.ecrire_journal("depart", {"x": depart[0], "y": depart[1],
                                    "dist_base": distance_base,
                                    "dans_zone": bool(env.lim(*depart))})
        if distance_base > 15.0:
            print("Attention : le bateau est a {:.0f} m de la base.".format(distance_base))

        s.definir_phase("DEPART", "depart dans {:.0f} s".format(opts["attente"]))
        s.compte_a_rebours(opts["attente"])
        depart, _ = s.position_fraiche()
        if depart is None:
            depart = np.array(env.home, dtype=float)

        s.definir_phase("ALLER_D1", "D1 a {:.0f} m".format(
            float(np.linalg.norm(self.d1 - depart))))
        issue = self._aller_d1(depart)
        s.ecrire_journal("aller_issue", {"issue": issue})

        if issue == "sortie":
            s.definir_phase("HORS_LIMITES", "sortie de zone detectee")
            s.pause(1.0)
            s.definir_phase("TOUR", "tour complet")
            tour_ok = ddb.tour_complet()
            s.ecrire_journal("tour_fin", {"reussi": bool(tour_ok)})
        else:
            print("Aller interrompu : {}".format(issue))

        s.definir_phase("RETOUR", "retour par le couloir" if issue == "sortie"
                        else "aller interrompu ({}), retour".format(issue))
        arrive = ddb.homing(env, rayon_arrivee=opts["rayon_arrivee"],
                            timeout=opts["timeout_retour"])
        if arrive:
            s.definir_phase("ARRIVEE", "base atteinte")
            return 0, ("arrivee" if issue == "sortie" else "arrivee_sans_sortie")
        s.definir_phase("ECHEC", "retour non termine")
        return 1, "retour_timeout"
