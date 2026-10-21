# -*- coding: utf-8 -*-
"""Essais de reglage et mission polygone, lances depuis l'IHM.

Essais (a faire avant la mission polygone, dans l'ordre) :

1. cap        : tient des caps fixes (N, E, S, O) ; mesure l'ecart de cap, le
                biais du compas et la derive.
2. segment    : aller-retour sur un segment, avec et sans compensation de
                derive ; mesure l'ecart lateral au segment.
3. point      : rallie un point (gradient ponctuel) puis revient.

Mission :

- polygone    : parcourt un polygone defini dans polygone.json, avec le
                gradient de ligne (segment courant) et le gradient ponctuel
                (sommet vise).

Chaque mission ecrit son journal dans logs/ (voir session.py). Les essais
logguent en plus un evenement de bilan par cap ou par segment.
"""

import json
import math
import time
from pathlib import Path

import numpy as np

from ddboatclass import angle_diff
from mission_d1 import MissionD1
from navigation import SuiveurChemin, construire_chemin_ferme, point_a_distance
from session import BASE_DIR, Mission

CHOIX_OUI_NON = ["oui", "non"]


def _param(nom, label, defaut, **autres):
    definition = {"nom": nom, "label": label, "defaut": defaut}
    definition.update(autres)
    return definition


class _MissionNavigation(Mission):
    """Fonctions communes : attente du GPS, compte a rebours, point de depart."""

    def _preparer_depart(self, attente):
        s = self.s
        s.definir_phase("ATTENTE_GPS", "en attente d'un fix GPS")
        depart = s.attendre_gps(120.0)
        if depart is None:
            s.definir_phase("ECHEC", "aucun fix GPS")
            return None
        s.ecrire_journal("depart", {"x": depart[0], "y": depart[1],
                                    "dist_base": float(np.linalg.norm(depart)),
                                    "dans_zone": bool(self.env.lim(*depart))})
        s.definir_phase("DEPART", "depart dans {:.0f} s".format(attente))
        s.compte_a_rebours(attente)
        position, _ = s.position_fraiche()
        return position if position is not None else depart


def _cap_demande(session, texte):
    """Cap du segment : valeur saisie, ou cap actuel du compas si vide."""
    texte = str(texte).strip()
    if texte:
        return float(texte)
    cap = session.ddb.dernier_cap
    if cap is None:
        cap = session.ddb.current_cap_rad()
    return math.degrees(cap)


# ----------------------------------------------------------------------
# Essai 1 : caps fixes
# ----------------------------------------------------------------------
class EssaiCap(_MissionNavigation):
    NOM = "cap"
    TITRE = "Essai 1 : caps fixes"
    DESCRIPTION = ("Tient successivement plusieurs caps (par defaut N, E, S, O, "
                   "25 s chacun). Verifie le suivi de cap et mesure le biais du "
                   "compas et la derive. Le bateau avance : prevoir 40 m d'eau "
                   "libre autour du depart.")
    PARAMETRES = [
        _param("caps", "Caps a tenir (deg, separes par des virgules)", "0,90,180,270",
               type="texte"),
        _param("duree", "Duree par cap (s)", 25.0),
        _param("derive", "Compensation de derive", "non", type="choix",
               choix=CHOIX_OUI_NON),
        _param("vitesse", "Vitesse (commande moteur)", 150, type="entier"),
        _param("rayon_securite", "Arret au-dela de (m du depart)", 80.0),
        _param("attente", "Compte a rebours (s)", 10.0),
    ]

    def executer(self):
        s, ddb, p = self.s, self.ddb, self.p
        try:
            caps = [float(v) for v in p["caps"].replace(";", ",").split(",") if v.strip()]
        except ValueError:
            return 1, "parametres"
        if not caps:
            return 1, "parametres"
        depart = self._preparer_depart(p["attente"])
        if depart is None:
            return 1, "pas_de_gps"

        derive = p["derive"] == "oui"
        bilan = []
        for cap_deg in caps:
            cap_rad = math.radians(cap_deg)
            s.definir_phase("ESSAI", "cap {:.0f}° ({})".format(
                cap_deg, "avec derive" if derive else "sans derive"))
            ddb.reinitialiser_asservissement()
            debut = time.monotonic()
            erreurs, cogs, vitesses = [], [], []
            dernier_fix = ddb.compteur_fix_gps
            while time.monotonic() - debut < p["duree"]:
                s.verifier_arret()
                position, age = s.position_fraiche()
                if position is None:
                    ddb._arreter_moteurs()
                    s.ecrire_journal("gps_perime", {"age": age})
                    return 1, "gps_perdu"
                if float(np.linalg.norm(position - depart)) > p["rayon_securite"]:
                    ddb._arreter_moteurs()
                    s.ecrire_journal("securite", {"x": position[0], "y": position[1]})
                    return 1, "securite"
                cible, cap, gauche, droite = ddb.suivre_route(
                    cap_rad, derive=derive, vitesse=p["vitesse"])
                erreur = math.degrees(angle_diff(cible, cap))
                sol = ddb.vitesse_sol or {}
                regime = time.monotonic() - debut > p["duree"] / 3.0
                if regime:
                    erreurs.append(erreur)
                    if ddb.compteur_fix_gps != dernier_fix and sol:
                        cogs.append(angle_diff(math.radians(sol["cog"]), cap_rad))
                        vitesses.append(sol["vitesse"])
                if ddb.compteur_fix_gps != dernier_fix:
                    dernier_fix = ddb.compteur_fix_gps
                s.ecrire_journal("nav", dict({
                    "mission": "cap", "x": position[0], "y": position[1],
                    "age": age, "cap_voulu": cap_deg,
                    "cap_cible": math.degrees(cible), "cap": math.degrees(cap),
                    "erreur": erreur, "moteurs": (gauche, droite),
                    "mag": ddb.dernier_mag_brut,
                    "v_sol": sol.get("vitesse"), "cog": sol.get("cog"),
                }, **ddb.dernier_asservissement))
                s.definir_message("erreur de cap {:+.0f}°".format(erreur))
                time.sleep(0.05)
            resume = {
                "cap_voulu": cap_deg, "derive": derive, "n": len(erreurs),
                "erreur_moy": float(np.mean(erreurs)) if erreurs else None,
                "erreur_std": float(np.std(erreurs)) if erreurs else None,
                "ecart_route_moy": math.degrees(float(np.mean(cogs))) if cogs else None,
                "v_sol_moy": float(np.mean(vitesses)) if vitesses else None,
            }
            estimation = ddb.derive.estimer(time.monotonic())
            if estimation:
                resume.update({"derive_est": estimation["est"],
                               "derive_nord": estimation["nord"],
                               "s_eau": estimation["s"]})
            s.ecrire_journal("cap_fin", resume)
            bilan.append(resume)
        ddb._arreter_moteurs()
        s.ecrire_journal("bilan", {"caps": bilan})
        s.definir_phase("TERMINE", "essai termine")
        return 0, "termine"


# ----------------------------------------------------------------------
# Essai 2 : segment aller-retour
# ----------------------------------------------------------------------
class EssaiSegment(_MissionNavigation):
    NOM = "segment"
    TITRE = "Essai 2 : suivi de segment"
    DESCRIPTION = ("Aller-retour sur un segment depuis la position de depart, "
                   "dans le cap choisi (vide = cap actuel). Mode alterne : la "
                   "premiere moitie des trajets sans compensation de derive, la "
                   "seconde avec. Le bilan donne l'ecart lateral au segment.")
    PARAMETRES = [
        _param("longueur", "Longueur du segment (m)", 40.0),
        _param("cap", "Cap du segment (deg, vide = cap actuel)", "", type="texte"),
        _param("trajets", "Nombre de trajets (aller ou retour)", 4, type="entier"),
        _param("derive", "Compensation de derive", "alterne", type="choix",
               choix=["alterne", "oui", "non"]),
        _param("rayon_arrivee", "Rayon d'arrivee (m)", 3.0),
        _param("vitesse", "Vitesse (commande moteur)", 150, type="entier"),
        _param("ecart_max", "Arret si ecart au segment > (m)", 25.0),
        _param("attente", "Compte a rebours (s)", 10.0),
    ]

    @classmethod
    def apercu(cls, session, parametres):
        position = session.position_actuelle()
        if position is None:
            return None
        cap = parametres["cap"] if parametres["cap"] != "" else position[2]
        if cap in (None, ""):
            return None
        return [[position[0], position[1]],
                point_a_distance(position, float(cap), parametres["longueur"])]

    def executer(self):
        s, ddb, p = self.s, self.ddb, self.p
        try:
            depart = self._preparer_depart(p["attente"])
            if depart is None:
                return 1, "pas_de_gps"
            cap_deg = _cap_demande(s, p["cap"])
        except ValueError:
            return 1, "parametres"
        a = [float(depart[0]), float(depart[1])]
        b = point_a_distance(a, cap_deg, p["longueur"])
        trajets = max(1, p["trajets"])
        s.ecrire_journal("segment", {"a": a, "b": b, "cap": cap_deg,
                                     "longueur": p["longueur"], "trajets": trajets})
        bilan = []
        for i in range(trajets):
            if p["derive"] == "oui":
                derive = True
            elif p["derive"] == "non":
                derive = False
            else:
                derive = i >= trajets // 2
            origine, cible = (a, b) if i % 2 == 0 else (b, a)
            s.definir_phase("ESSAI", "trajet {}/{} ({})".format(
                i + 1, trajets, "avec derive" if derive else "sans derive"))
            suiveur = SuiveurChemin([origine, cible], rayon_arrivee=p["rayon_arrivee"])
            issue, resume = s.suivre_chemin(
                suiveur, derive=derive, vitesse=p["vitesse"],
                ecart_max=p["ecart_max"], timeout=240.0, etiquette="segment")
            s.ecrire_journal("trajet_fin", {"trajet": i, "issue": issue,
                                            "derive": derive, "segments": resume})
            bilan.extend(resume)
            if issue != "termine":
                return 1, issue
        sans = [r["lateral_rms"] for r in bilan if not r["derive"] and r["n"] > 5]
        avec = [r["lateral_rms"] for r in bilan if r["derive"] and r["n"] > 5]
        s.ecrire_journal("bilan", {
            "lateral_rms_sans_derive": float(np.mean(sans)) if sans else None,
            "lateral_rms_avec_derive": float(np.mean(avec)) if avec else None})
        s.definir_phase("TERMINE", "essai termine")
        return 0, "termine"


# ----------------------------------------------------------------------
# Essai 3 : ralliement d'un point
# ----------------------------------------------------------------------
class EssaiPoint(_MissionNavigation):
    NOM = "point"
    TITRE = "Essai 3 : ralliement d'un point"
    DESCRIPTION = ("Rallie un point situe devant le bateau avec le gradient "
                   "ponctuel seul (comme le homing), puis revient au depart.")
    PARAMETRES = [
        _param("distance", "Distance du point (m)", 30.0),
        _param("cap", "Cap du point (deg, vide = cap actuel)", "", type="texte"),
        _param("aller_retour", "Revenir au depart", "oui", type="choix",
               choix=CHOIX_OUI_NON),
        _param("derive", "Compensation de derive", "oui", type="choix",
               choix=CHOIX_OUI_NON),
        _param("rayon_arrivee", "Rayon d'arrivee (m)", 3.0),
        _param("vitesse", "Vitesse (commande moteur)", 150, type="entier"),
        _param("ecart_max", "Arret si ecart a la route > (m)", 40.0),
        _param("attente", "Compte a rebours (s)", 10.0),
    ]

    @classmethod
    def apercu(cls, session, parametres):
        return EssaiSegment.apercu(session, {
            "cap": parametres["cap"], "longueur": parametres["distance"]})

    def executer(self):
        s, ddb, p = self.s, self.ddb, self.p
        try:
            depart = self._preparer_depart(p["attente"])
            if depart is None:
                return 1, "pas_de_gps"
            cap_deg = _cap_demande(s, p["cap"])
        except ValueError:
            return 1, "parametres"
        a = [float(depart[0]), float(depart[1])]
        cible = point_a_distance(a, cap_deg, p["distance"])
        s.ecrire_journal("point", {"a": a, "cible": cible, "cap": cap_deg})
        derive = p["derive"] == "oui"
        etapes = [("point", a, cible)]
        if p["aller_retour"] == "oui":
            etapes.append(("retour", cible, a))
        for nom, origine, but in etapes:
            s.definir_phase("ESSAI", "ralliement : {}".format(nom))
            debut = time.monotonic()
            # k_ligne = 0 : gradient ponctuel seul.
            suiveur = SuiveurChemin([origine, but], rayon_arrivee=p["rayon_arrivee"],
                                    k_ligne=0.0)
            issue, resume = s.suivre_chemin(
                suiveur, derive=derive, vitesse=p["vitesse"],
                ecart_max=p["ecart_max"], timeout=240.0, etiquette="point")
            position, _ = s.position_fraiche()
            s.ecrire_journal("point_fin", {
                "etape": nom, "issue": issue, "duree": time.monotonic() - debut,
                "dist_cible": (float(np.linalg.norm(position - np.array(but)))
                               if position is not None else None)})
            if issue != "termine":
                return 1, issue
        s.definir_phase("TERMINE", "essai termine")
        return 0, "termine"


# ----------------------------------------------------------------------
# Mission polygone
# ----------------------------------------------------------------------
def charger_polygone(session, fichier):
    """Lit le polygone : (points locaux [[x, y], ...], ferme, nom).

    Format de polygone.json :
        {"nom": "...", "ferme": true,
         "points_latlon": [[lat, lon], ...]}      (ou "points_xy" en metres)
    """
    chemin = Path(fichier)
    if not chemin.is_absolute():
        chemin = BASE_DIR / chemin
    with open(str(chemin), encoding="utf-8") as f:
        donnees = json.load(f)
    env = session.env
    if "points_latlon" in donnees:
        points = []
        for lat, lon in donnees["points_latlon"]:
            x, y = session.convertisseur.convert_cart_position(
                float(lat), float(lon), env.home_latlong, env.rotation)
            points.append([float(x), float(y)])
    elif "points_xy" in donnees:
        points = [[float(x), float(y)] for x, y in donnees["points_xy"]]
    else:
        raise ValueError("points_latlon ou points_xy attendu dans {}".format(chemin.name))
    ferme = bool(donnees.get("ferme", True))
    if len(points) < (3 if ferme else 2):
        raise ValueError("pas assez de points dans {}".format(chemin.name))
    return points, ferme, donnees.get("nom", chemin.stem)


class MissionPolygone(_MissionNavigation):
    NOM = "polygone"
    TITRE = "Mission : polygone"
    DESCRIPTION = ("Parcourt le polygone du fichier (polygone.json) : gradient de "
                   "ligne sur le segment courant et gradient ponctuel vers le "
                   "sommet vise, avec compensation de derive. Le premier segment "
                   "va de la position de depart au premier sommet.")
    PARAMETRES = [
        _param("fichier", "Fichier du polygone", "polygone.json", type="texte"),
        _param("tours", "Nombre de tours", 1, type="entier"),
        _param("derive", "Compensation de derive", "oui", type="choix",
               choix=CHOIX_OUI_NON),
        _param("rayon_arrivee", "Rayon d'arrivee aux sommets (m)", 3.0),
        _param("rayon_final", "Rayon d'arrivee au dernier sommet (m)", 6.0),
        _param("ralentir_fin", "Ralentir sur les derniers (m)", 15.0),
        _param("vitesse_fin", "Vitesse d'arrivee (commande moteur)", 90, type="entier"),
        _param("k_ligne", "Gain de ligne (1/m)", SuiveurChemin.K_LIGNE),
        _param("vitesse", "Vitesse (commande moteur)", 150, type="entier"),
        _param("ecart_max", "Arret si ecart au chemin > (m)", 40.0),
        _param("timeout", "Duree maximale (s)", 1800.0),
        _param("attente", "Compte a rebours (s)", 10.0),
    ]

    @classmethod
    def apercu(cls, session, parametres):
        points, ferme, _ = charger_polygone(session, parametres["fichier"])
        if ferme:
            return points + [points[0]]
        position = session.position_actuelle()
        return ([[position[0], position[1]]] if position else []) + points

    def executer(self):
        s, ddb, p = self.s, self.ddb, self.p
        try:
            sommets, ferme, nom = charger_polygone(s, p["fichier"])
        except (OSError, ValueError) as exc:
            s.definir_phase("ECHEC", "polygone : {}".format(exc))
            return 1, "polygone_invalide"
        s.ecrire_journal("polygone", {"nom": nom, "ferme": ferme,
                                      "sommets": sommets, "fichier": p["fichier"]})
        depart = self._preparer_depart(p["attente"])
        if depart is None:
            return 1, "pas_de_gps"
        a = [float(depart[0]), float(depart[1])]
        if ferme:
            chemin = construire_chemin_ferme(a, sommets, p["tours"])
        else:
            chemin = [a] + sommets
        suiveur = SuiveurChemin(chemin, rayon_arrivee=p["rayon_arrivee"],
                                k_ligne=p["k_ligne"], rayon_final=p["rayon_final"])
        s.ecrire_journal("chemin", {"points": chemin,
                                    "longueur": suiveur.longueur_totale()})
        s.definir_phase("NAVIGATION", "polygone « {} » : {} sommets, {:.0f} m".format(
            nom, len(sommets), suiveur.longueur_totale()))
        issue, resume = s.suivre_chemin(
            suiveur, derive=p["derive"] == "oui", vitesse=p["vitesse"],
            ecart_max=p["ecart_max"], timeout=p["timeout"], etiquette=self.NOM,
            ralentir_fin=p["ralentir_fin"], vitesse_fin=p["vitesse_fin"])
        position, _ = s.position_fraiche()
        s.ecrire_journal("bilan", {
            "issue": issue, "segments": resume,
            "dist_dernier_sommet": (float(np.linalg.norm(
                position - suiveur.points[-1])) if position is not None else None)})
        if issue != "termine":
            s.definir_phase("ECHEC", "polygone interrompu ({})".format(issue))
            return 1, issue
        s.definir_phase("TERMINE", "polygone termine")
        return 0, "termine"


class Mission3(MissionPolygone):
    NOM = "mission3"
    TITRE = "Mission 3 : A-C-D-G'-E-F-G-A"
    DESCRIPTION = ("Trajet A-C-D-G'-E-F-G-A (mission3.json) : gradient de ligne et "
                   "gradient ponctuel, avec compensation de derive. Part de la "
                   "base A, finit a A en ralentissant pour s'arreter avant le "
                   "ponton. 625 m, environ 9 minutes.")
    PARAMETRES = [dict(definition, defaut="mission3.json")
                  if definition["nom"] == "fichier" else definition
                  for definition in MissionPolygone.PARAMETRES
                  if definition["nom"] != "tours"]

    def __init__(self, session, parametres):
        parametres = dict(parametres, tours=1)
        super(Mission3, self).__init__(session, parametres)


MISSIONS = [MissionD1, EssaiCap, EssaiSegment, EssaiPoint, MissionPolygone, Mission3]


def enregistrer_missions(session):
    for classe in MISSIONS:
        session.enregistrer(classe)
