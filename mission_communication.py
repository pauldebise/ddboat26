# -*- coding: utf-8 -*-
"""Mission ABBA : deux bateaux echangent leur point de depart.

Chaque bateau part de son point et rejoint celui de l'autre, en l'evitant.
Les positions passent par le serveur UDP du lac (communication.py, fige) a
1 Hz ; le thread de communication de la Session les recoit en continu et la
mission lit `session.etat_autre()`.

Deroulement
-----------
1. ATTENTE_GPS   : attente d'un fix GPS.
2. ATTENTE_AUTRE : attente d'une position fraiche de l'autre (60 s au plus).
3. DEPART        : compte a rebours, bateau immobile.
4. NAVIGATION    : gradient de ligne (depart -> arrivee) + gradient ponctuel
                   (arrivee) + repulsion de l'autre bateau.
5. TERMINE       : moteurs coupes a moins de rayon_arrivee de l'arrivee, si
                   le but est libre.

Securites (moteurs coupes) : autre bateau a moins de rayon_urgence (reprise
au-dela de 8 m), position de l'autre trop vieille ou absente, GPS perime,
duree maximale depassee. Dans la zone d'evitement le bateau ralentit.

Le choix du cote d'evitement n'est pas impose : le bruit du GPS depart age
les face-a-face. Un seul moteur peut etre en arriere, jamais les deux.
"""

import math
import time

import numpy as np

from ddboatclass import angle_diff
from navigation import SuiveurChemin
from session import Mission

CHOIX_OUI_NON = ["oui", "non"]

# Repulsion : k_rep * (1/d - 1/rayon). Egale au gradient ponctuel (norme 1)
# vers 8 m pour un rayon de 15 m, puis de plus en plus forte, plafonnee.
K_REP = 17.0
PLAFOND_REP = 3.0
D_MIN_REP = 1.5            # distance minimale utilisee dans la formule (m)
FACTEUR_MIN = 0.6          # facteur de vitesse au rayon d'urgence
REPRISE_URGENCE = 8.0      # arret d'urgence leve au-dela de cette distance (m)
HYST_BUT = 2.0             # le but est libre a rayon_but_occupe + HYST_BUT (m)
ARRET_ATTENTE = 2.0        # arret a moins de ... m du point d'attente
REPART_ATTENTE = 3.0       # le bateau repart au-dela de ... m du point d'attente
ATTENTE_AUTRE_MAX = 60.0   # attente d'une position fraiche de l'autre (s)
K_LIGNE = SuiveurChemin.K_LIGNE


def _param(nom, label, defaut, **autres):
    definition = {"nom": nom, "label": label, "defaut": defaut}
    definition.update(autres)
    return definition


def calculer_repulsion(position, autre, rayon, k_rep=K_REP, plafond=PLAFOND_REP,
                       d_min=D_MIN_REP):
    """Vecteur de repulsion de l'autre bateau. Retourne (vecteur, distance).

    Nul a rayon et au-dela ; en dessous, k_rep * (1/d - 1/rayon) dirige de
    l'autre vers nous, avec d >= d_min et une norme plafonnee. Ne depend
    que des positions : aucun cote n'est prefere.
    """
    ecart = np.asarray(position, dtype=float) - np.asarray(autre, dtype=float)
    distance = float(math.hypot(ecart[0], ecart[1]))
    if distance >= rayon or distance < 1e-6:
        return np.zeros(2), distance
    d = max(distance, d_min)
    norme = min(plafond, k_rep * (1.0 / d - 1.0 / rayon))
    return norme * ecart / distance, distance


def facteur_vitesse(distance, rayon_urgence, rayon_evitement):
    """1 hors de la zone d'evitement, FACTEUR_MIN au rayon d'urgence."""
    if rayon_evitement <= rayon_urgence:
        return 1.0
    t = (distance - rayon_urgence) / (rayon_evitement - rayon_urgence)
    return FACTEUR_MIN + (1.0 - FACTEUR_MIN) * max(0.0, min(1.0, t))


def borner_marche_arriere(gauche, droite):
    """Un seul moteur en arriere : si les deux le sont, le moins negatif passe a 0."""
    if gauche < 0 and droite < 0:
        if gauche >= droite:
            return 0, droite
        return gauche, 0
    return gauche, droite


class MissionABBA(Mission):
    NOM = "abba"
    TITRE = "Mission ABBA (échange de points, évitement)"
    DESCRIPTION = ("Points en mode auto : depart = ma position au lancement, arrivee = "
                   "position de l'autre bateau a ce moment. Sinon, rejoint le point de depart de l'autre bateau (arrivee) depuis le "
                   "depart choisi, en l'evitant : repulsion dans un rayon de 15 m, "
                   "ralentissement, arret d'urgence sous 5 m, arret si sa position "
                   "est trop vieille, attente a 10 m si le but est occupe. Les "
                   "positions passent par le serveur du lac (panneau Communication).")
    PARAMETRES = [
        _param("points", "Points depart/arrivee", "choisis", type="choix",
               choix=["choisis", "auto"]),
        _param("depart_lat", "Depart : latitude (deg)", 48.198863),
        _param("depart_lon", "Depart : longitude (deg)", -3.015070),
        _param("arrivee_lat", "Arrivee : latitude (deg)", 48.199072),
        _param("arrivee_lon", "Arrivee : longitude (deg)", -3.016258),
        _param("rayon_evitement", "Rayon d'evitement (m)", 15.0),
        _param("rayon_urgence", "Arret d'urgence sous (m)", 5.0),
        _param("age_max", "Position de l'autre trop vieille apres (s)", 3.0),
        _param("vitesse", "Vitesse (commande moteur)", 150, type="entier"),
        _param("rayon_arrivee", "Rayon d'arrivee (m)", 3.0),
        _param("rayon_but_occupe", "But occupe si l'autre a moins de (m)", 10.0),
        _param("distance_attente", "Attente du but a (m)", 10.0),
        _param("ralentir_fin", "Ralentir sur les derniers (m)", 10.0),
        _param("vitesse_fin", "Vitesse d'arrivee (commande moteur)", 90, type="entier"),
        _param("timeout", "Duree maximale (s)", 600.0),
        _param("attente", "Compte a rebours (s)", 10.0),
        _param("derive", "Compensation de derive", "oui", type="choix",
               choix=CHOIX_OUI_NON),
    ]

    # ------------------------------------------------------------------
    # Points
    # ------------------------------------------------------------------
    @staticmethod
    def _local(session, lat, lon):
        env = session.env
        x, y = session.convertisseur.convert_cart_position(
            lat, lon, env.home_latlong, env.rotation)
        return np.array([float(x), float(y)])

    @classmethod
    def apercu(cls, session, parametres):
        if parametres["points"] == "auto":
            position = session.position_actuelle()
            autre = session.etat_autre()
            if position is None or autre is None:
                return None
            return [[position[0], position[1]], [autre["x"], autre["y"]]]
        depart = cls._local(session, parametres["depart_lat"], parametres["depart_lon"])
        arrivee = cls._local(session, parametres["arrivee_lat"], parametres["arrivee_lon"])
        return [depart, arrivee]

    def _verifier_parametres(self, depart, arrivee):
        p = self.p
        if p["points"] != "auto":
            for nom in ("depart_lat", "arrivee_lat"):
                if abs(p[nom]) > 90.0:
                    return "latitude invalide"
            for nom in ("depart_lon", "arrivee_lon"):
                if abs(p[nom]) > 180.0:
                    return "longitude invalide"
        if float(np.linalg.norm(arrivee - depart)) < 5.0:
            return "depart et arrivee a moins de 5 m"
        if not 0.0 < p["rayon_urgence"] < p["rayon_evitement"]:
            return "il faut 0 < rayon d'urgence < rayon d'evitement"
        if REPRISE_URGENCE <= p["rayon_urgence"]:
            return "le rayon d'urgence doit rester sous {:.0f} m".format(REPRISE_URGENCE)
        if not 0 < p["vitesse_fin"] <= p["vitesse"]:
            return "il faut 0 < vitesse d'arrivee <= vitesse"
        if p["rayon_arrivee"] <= 0.0 or p["distance_attente"] <= p["rayon_arrivee"]:
            return "il faut distance d'attente > rayon d'arrivee > 0"
        if p["age_max"] <= 0.0 or p["timeout"] <= 0.0:
            return "age maximal et duree maximale doivent etre positifs"
        return None

    # ------------------------------------------------------------------
    # Deroulement
    # ------------------------------------------------------------------
    def executer(self):
        s, p = self.s, self.p
        auto = p["points"] == "auto"
        if not auto:
            depart_ref = self._local(s, p["depart_lat"], p["depart_lon"])
            arrivee = self._local(s, p["arrivee_lat"], p["arrivee_lon"])
            erreur = self._verifier_parametres(depart_ref, arrivee)
            if erreur:
                s.definir_phase("ECHEC", erreur)
                return 1, "parametres"

        s.definir_phase("ATTENTE_GPS", "en attente d'un fix GPS")
        depart = s.attendre_gps(120.0)
        if depart is None:
            s.definir_phase("ECHEC", "aucun fix GPS")
            return 1, "pas_de_gps"
        autre = self._attendre_autre()
        if autre is None:
            return 1, "pas_de_lien"
        if auto:
            # Points releves au lancement : ma position et celle de l'autre.
            depart = s.position_fraiche()[0]
            if depart is None:
                s.definir_phase("ECHEC", "aucun fix GPS")
                return 1, "pas_de_gps"
            depart_ref = np.array([float(depart[0]), float(depart[1])])
            arrivee = np.array([autre["x"], autre["y"]])
            erreur = self._verifier_parametres(depart_ref, arrivee)
            if erreur:
                s.definir_phase("ECHEC", erreur)
                return 1, "parametres"
            s.ecrire_journal("points_auto", {
                "depart_ref": depart_ref, "arrivee": arrivee,
                "depart_latlon": s.ddb.derniere_position_gps,
                "arrivee_latlon": (autre["lat"], autre["lon"]),
                "age_autre": autre["age"]})

        s.ecrire_journal("depart", {
            "x": depart[0], "y": depart[1], "cap": self._cap_deg(),
            "dist_arrivee": float(np.linalg.norm(depart - arrivee)),
            "dist_depart_choisi": float(np.linalg.norm(depart - depart_ref)),
            "dist_base": float(np.linalg.norm(depart)),
            "dans_zone": bool(self.env.lim(*depart)),
            "depart_ref": depart_ref, "arrivee": arrivee, "points": p["points"],
            "autre_x": autre["x"], "autre_y": autre["y"],
            "autre_age": autre["age"],
            "communication": s.configuration_communication()})
        s.definir_phase("DEPART", "depart dans {:.0f} s".format(p["attente"]))
        s.compte_a_rebours(p["attente"])
        return self._naviguer(depart_ref, arrivee)

    def _cap_deg(self):
        cap = self.ddb.dernier_cap
        return math.degrees(cap) if cap is not None else None

    def _autre_frais(self):
        """Etat de l'autre bateau s'il est assez recent, sinon None."""
        autre = self.s.etat_autre()
        if autre is None or autre["age"] > self.p["age_max"]:
            return None
        return autre

    def _attendre_autre(self):
        s = self.s
        config = s.configuration_communication()
        if not config["ip"]:
            s.definir_phase("ECHEC", "communication non configuree (IP du serveur)")
            return None
        s.definir_phase("ATTENTE_AUTRE", "en attente de l'autre bateau (id {})".format(
            config["autre_id"]))
        fin = time.monotonic() + ATTENTE_AUTRE_MAX
        while time.monotonic() < fin:
            s.verifier_arret()
            autre = self._autre_frais()
            if autre is not None:
                return autre
            time.sleep(0.2)
        s.ecrire_journal("pas_de_lien", {"attente": ATTENTE_AUTRE_MAX})
        s.definir_phase("ECHEC", "aucune position de l'autre bateau")
        return None

    def _phase(self, phase, message=""):
        """Change de phase seulement si elle differe (une ligne par transition)."""
        s = self.s
        if s.phase != phase:
            s.definir_phase(phase, message)
        elif message:
            s.definir_message(message)

    def _noter(self, etat, position, age, dt, autre, d_autre, d_arrivee, **champs):
        donnees = {
            "mission": self.NOM, "etat": etat,
            "x": position[0], "y": position[1], "age": age, "dt": dt,
            "dist_arrivee": d_arrivee, "dist_autre": d_autre,
            "age_autre": autre["age"], "autre_x": autre["x"], "autre_y": autre["y"],
            "autre_cap": autre["cap"],
        }
        donnees.update(champs)
        self.s.ecrire_journal("nav", donnees)

    def _naviguer(self, depart_ref, arrivee):
        s, ddb, p = self.s, self.ddb, self.p
        derive = p["derive"] == "oui"
        suiveur = SuiveurChemin([depart_ref, arrivee], rayon_arrivee=p["rayon_arrivee"],
                                k_ligne=K_LIGNE)
        ddb.reinitialiser_asservissement()
        debut = time.monotonic()
        precedent = debut
        drapeaux = {"gps_perime": False, "lien_perdu": False, "urgence": False,
                    "evitement": False, "occupe": False, "attente": False}
        bilan = {"dist_min_autre": None, "duree_evitement": 0.0, "arrets_urgence": 0,
                 "duree_urgence": 0.0, "duree_lien_perdu": 0.0, "duree_but_occupe": 0.0,
                 "double_marche_arriere": 0, "resultat": None}
        direction_prec = None
        vecteurs_nuls = 0
        issue = "termine"
        s.definir_guidage(
            chemin=[[float(depart_ref[0]), float(depart_ref[1])],
                    [float(arrivee[0]), float(arrivee[1])]],
            a=[float(depart_ref[0]), float(depart_ref[1])],
            b=[float(arrivee[0]), float(arrivee[1])],
            zones=[p["rayon_arrivee"], p["rayon_but_occupe"]],
            n_segments=1, derive=derive)
        self._phase("NAVIGATION", "vers l'arrivee")
        try:
            while True:
                s.verifier_arret()
                maintenant = time.monotonic()
                dt = maintenant - precedent
                precedent = maintenant
                if maintenant - debut > p["timeout"]:
                    ddb._arreter_moteurs()
                    s.ecrire_journal("timeout", {"duree": maintenant - debut})
                    issue = "timeout"
                    break

                position, age = s.position_fraiche()
                if position is None:
                    ddb._arreter_moteurs()
                    if not drapeaux["gps_perime"]:
                        s.ecrire_journal("gps_perime", {"age": age})
                        s.definir_message("GPS perdu, moteurs coupes")
                        drapeaux["gps_perime"] = True
                    time.sleep(0.2)
                    continue
                if drapeaux["gps_perime"]:
                    s.ecrire_journal("gps_retabli", {"age": age})
                    drapeaux["gps_perime"] = False
                    ddb.reinitialiser_asservissement()

                # --- lien avec l'autre bateau -------------------------------
                autre = self._autre_frais()
                if autre is None:
                    ddb._arreter_moteurs()
                    bilan["duree_lien_perdu"] += dt
                    brut = s.etat_autre()
                    if not drapeaux["lien_perdu"]:
                        drapeaux["lien_perdu"] = True
                        s.ecrire_journal("lien_perdu", {
                            "age_autre": brut["age"] if brut else None,
                            "x": position[0], "y": position[1]})
                    self._phase("LIEN_PERDU", "position de l'autre trop vieille ou absente")
                    if brut is not None:
                        pos_brute = np.array([brut["x"], brut["y"]])
                        self._noter("lien_perdu", position, age, dt, brut,
                                    float(np.linalg.norm(position - pos_brute)),
                                    float(np.linalg.norm(position - arrivee)),
                                    moteurs=(0, 0))
                    time.sleep(0.1)
                    continue
                if drapeaux["lien_perdu"]:
                    drapeaux["lien_perdu"] = False
                    s.ecrire_journal("lien_retabli", {"age_autre": autre["age"]})
                    ddb.reinitialiser_asservissement()

                pos_autre = np.array([autre["x"], autre["y"]])
                d_autre = float(np.linalg.norm(position - pos_autre))
                d_arrivee = float(np.linalg.norm(position - arrivee))
                if bilan["dist_min_autre"] is None or d_autre < bilan["dist_min_autre"]:
                    bilan["dist_min_autre"] = d_autre
                cap_relatif = math.degrees(angle_diff(
                    math.atan2(pos_autre[0] - position[0], pos_autre[1] - position[1]),
                    ddb.dernier_cap if ddb.dernier_cap is not None else 0.0))

                # --- arret d'urgence (reprise a REPRISE_URGENCE) ---------------
                if d_autre < p["rayon_urgence"] or (
                        drapeaux["urgence"] and d_autre <= REPRISE_URGENCE):
                    ddb._arreter_moteurs()
                    bilan["duree_urgence"] += dt
                    if not drapeaux["urgence"]:
                        drapeaux["urgence"] = True
                        bilan["arrets_urgence"] += 1
                        s.ecrire_journal("arret_urgence", {
                            "x": position[0], "y": position[1], "dist_autre": d_autre,
                            "age_autre": autre["age"],
                            "autre_x": autre["x"], "autre_y": autre["y"]})
                    self._phase("ARRET_URGENCE", "autre bateau a {:.1f} m".format(d_autre))
                    self._noter("urgence", position, age, dt, autre, d_autre, d_arrivee,
                                cap_relatif=cap_relatif, moteurs=(0, 0))
                    s.definir_guidage(dist_cible=d_arrivee)
                    time.sleep(0.1)
                    continue
                if drapeaux["urgence"]:
                    drapeaux["urgence"] = False
                    s.ecrire_journal("reprise_urgence", {"dist_autre": d_autre})
                    ddb.reinitialiser_asservissement()

                # --- but occupe ? (hysteresis de HYST_BUT) --------------------
                d_autre_but = float(np.linalg.norm(pos_autre - arrivee))
                seuil = p["rayon_but_occupe"] + (HYST_BUT if drapeaux["occupe"] else 0.0)
                occupe = d_autre_but < seuil
                if occupe != drapeaux["occupe"]:
                    drapeaux["occupe"] = occupe
                    drapeaux["attente"] = False
                    s.ecrire_journal("but_occupe" if occupe else "but_occupe_fin", {
                        "x": position[0], "y": position[1],
                        "dist_autre_but": d_autre_but, "dist_arrivee": d_arrivee})
                    if not occupe:
                        ddb.reinitialiser_asservissement()
                if occupe:
                    bilan["duree_but_occupe"] += dt

                # --- guidage -------------------------------------------------
                if occupe:
                    # Point d'attente : sur la droite arrivee -> bateau, a
                    # distance_attente de l'arrivee ; gradient ponctuel seul.
                    u = position - arrivee
                    n = float(np.linalg.norm(u))
                    if n < 1e-6:
                        u = depart_ref - arrivee
                        n = float(np.linalg.norm(u))
                    attente_pt = arrivee + p["distance_attente"] * u / n
                    vers = attente_pt - position
                    dist_cible = float(np.linalg.norm(vers))
                    guidage = vers / dist_cible if dist_cible > 1e-6 else np.zeros(2)
                    limite = REPART_ATTENTE if drapeaux["attente"] else ARRET_ATTENTE
                    drapeaux["attente"] = dist_cible <= limite
                    rayon_cible = ARRET_ATTENTE
                    lateral = None
                else:
                    info = suiveur.evaluer(position)
                    guidage = np.asarray(info["direction"], dtype=float)
                    dist_cible = d_arrivee
                    drapeaux["attente"] = False
                    rayon_cible = p["rayon_arrivee"]
                    lateral = float(info["lateral"])
                    if d_arrivee < p["rayon_arrivee"]:
                        ddb._arreter_moteurs()
                        s.ecrire_journal("arrivee", {
                            "x": position[0], "y": position[1],
                            "dist_arrivee": d_arrivee, "dist_autre": d_autre,
                            "age_autre": autre["age"], "duree": maintenant - debut})
                        break

                repulsion, _ = calculer_repulsion(position, pos_autre, p["rayon_evitement"])
                rep_norme = float(np.linalg.norm(repulsion))
                actif = rep_norme > 0.0
                if actif != drapeaux["evitement"]:
                    drapeaux["evitement"] = actif
                    s.ecrire_journal("evitement_debut" if actif else "evitement_fin", {
                        "x": position[0], "y": position[1], "dist_autre": d_autre,
                        "age_autre": autre["age"]})
                if actif:
                    bilan["duree_evitement"] += dt

                if drapeaux["attente"]:
                    # Arrive au point d'attente : moteurs coupes, on regarde.
                    ddb._arreter_moteurs()
                    self._phase("BUT_OCCUPE", "attente a {:.1f} m du but".format(d_arrivee))
                    self._noter("attente_but", position, age, dt, autre, d_autre, d_arrivee,
                                dist_autre_but=d_autre_but, cap_relatif=cap_relatif,
                                repulsion=repulsion, moteurs=(0, 0))
                    s.definir_guidage(dist_cible=d_arrivee)
                    time.sleep(0.1)
                    continue

                total = guidage + repulsion
                nul = float(np.linalg.norm(total)) < 1e-3
                if nul:
                    # Guidage et repulsion s'annulent : on garde le cap precedent
                    # (atan2(0, 0) donnerait le nord).
                    vecteurs_nuls += 1
                    total = (direction_prec if direction_prec is not None
                             else arrivee - position)
                else:
                    direction_prec = total

                facteur = facteur_vitesse(d_autre, p["rayon_urgence"], p["rayon_evitement"])
                vitesse = p["vitesse"] * facteur
                if p["ralentir_fin"] > 0.0:
                    restant = max(0.0, dist_cible - rayon_cible)
                    v_arrivee = p["vitesse_fin"] + (p["vitesse"] - p["vitesse_fin"]) * min(
                        1.0, restant / p["ralentir_fin"])
                    vitesse = min(vitesse, v_arrivee)

                cible, cap, gauche, droite = ddb.maintien_cap(
                    total, self.env.rotation, derive=derive, vitesse=vitesse)
                corrige = borner_marche_arriere(gauche, droite)
                if corrige != (gauche, droite):
                    bilan["double_marche_arriere"] += 1
                    s.ecrire_journal("double_marche_arriere", {
                        "commande": (gauche, droite), "corrigee": corrige})
                    gauche, droite = corrige
                    ddb._commander_moteurs(gauche, droite)

                self._phase("EVITEMENT" if actif else ("BUT_OCCUPE" if occupe else "NAVIGATION"),
                            "autre bateau a {:.0f} m, but a {:.0f} m".format(d_autre, d_arrivee))
                asserv = ddb.dernier_asservissement
                sol = ddb.vitesse_sol or {}
                self._noter(
                    "evitement" if actif else ("but_occupe" if occupe else "navigation"),
                    position, age, dt, autre, d_autre, d_arrivee,
                    dist_autre_but=d_autre_but, cap_relatif=cap_relatif,
                    guidage=guidage, repulsion=repulsion, rep_norme=rep_norme,
                    vecteur_nul=nul, facteur=facteur, vitesse_voulue=vitesse,
                    cap_cible=math.degrees(cible), cap=math.degrees(cap),
                    erreur=math.degrees(angle_diff(cible, cap)),
                    moteurs=(gauche, droite), mag=ddb.dernier_mag_brut,
                    v_sol=sol.get("vitesse"), cog=sol.get("cog"), **asserv)
                norme = float(np.linalg.norm(total))
                champs_guidage = {
                    "dist_cible": d_arrivee,
                    "direction": ([float(total[0] / norme), float(total[1] / norme)]
                                  if norme > 1e-9 else [0.0, 0.0]),
                    "route": asserv.get("route"), "crabe": asserv.get("crabe")}
                if lateral is not None:
                    champs_guidage["lateral"] = lateral
                s.definir_guidage(**champs_guidage)
                time.sleep(0.05)
        finally:
            try:
                ddb._arreter_moteurs()
            except Exception:
                pass
            bilan["resultat"] = issue
            bilan["duree"] = time.monotonic() - debut
            bilan["vecteurs_nuls"] = vecteurs_nuls
            s.ecrire_journal("bilan", bilan)
            s.definir_guidage()
        if issue != "termine":
            s.definir_phase("ECHEC", "mission interrompue ({})".format(issue))
            return 1, issue
        s.definir_phase("TERMINE", "arrivee atteinte")
        return 0, "termine"
