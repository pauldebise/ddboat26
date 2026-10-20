# -*- coding: utf-8 -*-
"""Mission D1 : aller vers D1, sortie de zone, tour complet, retour au couloir.

Ce module est lance par l'IHM, dans le meme programme :

    python3 ihm_lac.py --mission

Deroulement
-----------
1. ATTENTE_GPS : attente d'un fix GPS (bateau pose a la base).
2. DEPART      : compte a rebours (--attente), le cap est enregistre.
3. ALLER_D1    : navigation vers D1 (premier point de la bordure droite).
                 A moins de --rayon-d1 metres de D1, le bateau garde le cap
                 d'approche pour franchir la bordure.
4. HORS_LIMITES: sortie confirmee sur --confirmations fix GPS successifs.
5. TOUR        : tour complet de 360 degres.
6. RETOUR      : retour a la base par le couloir (Environnement.direction()).
7. ARRIVEE     : moteurs coupes a moins de --rayon-arrivee metres de la base.

Journal
-------
Chaque mission ecrit un fichier logs/mission_d1_AAAAMMJJ_HHMMSS.jsonl : une
ligne JSON par evenement (configuration, phases, fix GPS, commandes de cap,
tour complet, homing, erreurs avec trace). Le fichier est aussi
telechargeable depuis l'IHM : http://<IP_BATEAU>:8000/journal
"""

import datetime
import json
import math
import os
import platform
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from pathlib import Path

import numpy as np

BASE_DIR = Path(__file__).resolve().parent
DRIVERS_DIR = BASE_DIR / "drivers"
if str(DRIVERS_DIR) not in sys.path:
    sys.path.insert(0, str(DRIVERS_DIR))

from ddboatclass import DDBoat, dist_cart, angle_diff  # noqa: E402

AGE_FIX_MAX = 3.0  # au-dela, la position GPS est consideree perimee (s)

# Valeurs par defaut des options de mission (surchargees par ihm_lac.py).
OPTIONS_DEFAUT = {
    "gps": 0,
    "attente": 10.0,          # compte a rebours avant le depart (s)
    "rayon_d1": 10.0,         # distance a D1 ou l'on garde le cap d'approche (m)
    "depassement_max": 20.0,  # distance max au-dela de D1 sans detection (m)
    "confirmations": 3,       # fix successifs hors limites pour valider
    "timeout_gps": 120.0,     # attente maximale du premier fix (s)
    "timeout_aller": 400.0,   # duree maximale de l'aller (s)
    "distance_max": 250.0,    # distance maximale a la base pendant l'aller (m)
    "timeout_retour": 900.0,  # duree maximale du retour (s)
    "rayon_arrivee": 5.0,     # distance a la base consideree atteinte (m)
}


def _json_defaut(objet):
    """Rend serialisables les types numpy et les objets inconnus."""
    if hasattr(objet, "tolist"):
        return _arrondir(objet.tolist())
    return repr(objet)


def _arrondir(valeur, cle=None):
    """Limite les chiffres significatifs pour alleger le journal.

    lat/lon gardent 10 chiffres (precision centimetrique), le reste 6.
    """
    if isinstance(valeur, float):
        if cle in ("lat", "lon", "home_latlong"):
            return float("%.10g" % valeur)
        return float("%.6g" % valeur)
    if isinstance(valeur, dict):
        return {k: _arrondir(v, k) for k, v in valeur.items()}
    if isinstance(valeur, (list, tuple)):
        return [_arrondir(v, cle) for v in valeur]
    if hasattr(valeur, "tolist"):
        return _arrondir(valeur.tolist(), cle)
    return valeur


class Journal(object):
    """Journal JSON Lines, ecrit depuis plusieurs threads."""

    # Types tres frequents : le fichier n'est vide sur disque qu'une fois
    # par seconde pour eux, immediatement pour tous les autres.
    FREQUENTS = ("nav", "homing", "tour", "fix")

    def __init__(self, dossier, prefixe="mission_d1"):
        dossier = Path(dossier)
        dossier.mkdir(parents=True, exist_ok=True)
        nom = "{}_{}.jsonl".format(
            prefixe, datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        self.chemin = dossier / nom
        self.fichier = open(str(self.chemin), "a", encoding="utf-8")
        self.verrou = threading.Lock()
        self.t0 = time.monotonic()
        self.dernier_flush = self.t0

    def ecrire(self, type_evenement, donnees=None):
        ligne = {
            "t": round(time.monotonic() - self.t0, 3),
            "heure": datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "type": type_evenement,
        }
        if donnees:
            ligne.update(_arrondir(donnees))
        texte = json.dumps(ligne, ensure_ascii=False, default=_json_defaut)
        with self.verrou:
            if self.fichier is None:
                return
            self.fichier.write(texte + "\n")
            maintenant = time.monotonic()
            if (type_evenement not in self.FREQUENTS
                    or maintenant - self.dernier_flush > 1.0):
                self.fichier.flush()
                try:
                    os.fsync(self.fichier.fileno())
                except OSError:
                    pass
                self.dernier_flush = maintenant

    def __call__(self, type_evenement, donnees):
        # Signature attendue par DDBoat.journal.
        self.ecrire(type_evenement, donnees)

    def fermer(self):
        with self.verrou:
            if self.fichier is not None:
                self.fichier.flush()
                self.fichier.close()
                self.fichier = None


def distances_bordures(env, x, y):
    """Distances signees aux bordures (> 0 = exterieur), comme lim().

    Uniquement pour le journal : elles montrent de combien le bateau est
    a l'interieur ou a l'exterieur de chaque bordure.
    """
    point = np.array([x, y], dtype=float)
    u = np.array(env.couloir[1] - env.couloir[0], dtype=float)
    u /= np.linalg.norm(u)
    normale_droite = np.array([u[1], -u[0]])

    def distance(bordure, attendue):
        a = np.array(bordure[0], dtype=float)
        v = np.array(bordure[1], dtype=float) - a
        v /= np.linalg.norm(v)
        normale = np.array([-v[1], v[0]])
        if np.dot(normale, attendue) < 0.0:
            normale = -normale
        return float(np.dot(point - a, normale))

    return (distance(env.bordure_droite, normale_droite),
            distance(env.bordure_gauche, -normale_droite))


def _commit_git():
    try:
        sortie = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=str(BASE_DIR),
            stderr=subprocess.DEVNULL)
        modifie = subprocess.check_output(
            ["git", "status", "--porcelain"], cwd=str(BASE_DIR),
            stderr=subprocess.DEVNULL)
        commit = sortie.decode().strip()
        return commit + ("+modifs" if modifie.strip() else "")
    except Exception:
        return None


class MissionD1(object):
    """Mission complete. executer() doit tourner dans le thread principal
    pour que Ctrl+C coupe les moteurs (gestionnaire de ArduinoIO)."""

    PHASES = {
        "INIT": "Initialisation",
        "ATTENTE_GPS": "Attente GPS",
        "DEPART": "Depart",
        "ALLER_D1": "Aller vers D1",
        "HORS_LIMITES": "Hors limites detecte",
        "TOUR": "Tour complet",
        "RETOUR": "Retour par le couloir",
        "ARRIVEE": "Base atteinte",
        "ECHEC": "Echec",
        "ARRET": "Arret",
    }

    def __init__(self, env, options=None):
        self.env = env
        self.options = dict(OPTIONS_DEFAUT)
        if options:
            self.options.update(options)
        self.d1 = np.array(env.bordure_droite[0], dtype=float)
        self.ddb = None
        self.verrou = threading.Lock()
        self.phase = "INIT"
        self.message = ""
        self.trail = deque(maxlen=1500)
        self.stop_gps = threading.Event()
        self.thread_gps = None
        self.journal = Journal(BASE_DIR / "logs")
        self.termine = False

    # ------------------------------------------------------------------
    # Etat partage avec l'IHM
    # ------------------------------------------------------------------
    def definir_phase(self, phase, message=""):
        with self.verrou:
            self.phase = phase
            self.message = message
        print("[{}] {}".format(phase, message))
        self.journal.ecrire("phase", {"phase": phase, "message": message})

    def definir_message(self, message):
        with self.verrou:
            self.message = message

    def _position(self):
        """Derniere position locale (x, y) sans lire le materiel."""
        if self.ddb is None or self.ddb.derniere_position_gps is None:
            return None
        lat, lon = self.ddb.derniere_position_gps
        return self.ddb.convert_cart_position(
            lat, lon, self.env.home_latlong, self.env.rotation)

    def _moteurs(self):
        ard = self.ddb.ard if self.ddb is not None else None
        if ard is None:
            return None
        try:
            gauche = -ard.cmdl if ard.dirl == "-" else ard.cmdl
            droite = -ard.cmdr if ard.dirr == "-" else ard.cmdr
            return [int(gauche), int(droite)]
        except AttributeError:
            return None

    def etat(self):
        """Etat courant pour l'IHM (appele depuis le thread du serveur)."""
        with self.verrou:
            phase = self.phase
            message = self.message
            trail = list(self.trail)
        ddb = self.ddb
        cap = ddb.dernier_cap if ddb is not None else None
        cap_cible = ddb.dernier_cap_cible if ddb is not None else None
        etat = {
            "phase": phase,
            "phase_texte": self.PHASES.get(phase, phase),
            "message": message,
            "x": None,
            "y": None,
            "lat": None,
            "lon": None,
            "fix_age": ddb.age_fix_gps() if ddb is not None else None,
            "dans_zone": None,
            "cap": math.degrees(cap) if cap is not None else None,
            "cap_cible": math.degrees(cap_cible) if cap_cible is not None else None,
            # Angle mathematique (0 = axe x, sens anti-horaire) pour le dessin.
            "heading": (math.pi / 2 - cap - self.env.rotation) if cap is not None else None,
            "moteurs": self._moteurs(),
            "trail": trail,
            "journal": str(self.journal.chemin.relative_to(BASE_DIR)),
        }
        position = self._position()
        if position is not None:
            x, y = float(position[0]), float(position[1])
            lat, lon = ddb.derniere_position_gps
            etat.update({"x": x, "y": y, "lat": lat, "lon": lon,
                         "dans_zone": bool(self.env.lim(x, y))})
        return etat

    # ------------------------------------------------------------------
    # Lecture GPS continue (y compris pendant le tour complet)
    # ------------------------------------------------------------------
    def _boucle_gps(self):
        dernier_compteur = 0
        while not self.stop_gps.wait(0.1):
            try:
                self.ddb.mise_a_jour_gps()
                if self.ddb.compteur_fix_gps == dernier_compteur:
                    continue
                dernier_compteur = self.ddb.compteur_fix_gps
                x, y = self._position()
                d_droite, d_gauche = distances_bordures(self.env, x, y)
                with self.verrou:
                    if (not self.trail or math.hypot(
                            x - self.trail[-1][0], y - self.trail[-1][1]) >= 0.7):
                        self.trail.append([round(x, 1), round(y, 1)])
                    phase = self.phase
                lat, lon = self.ddb.derniere_position_gps
                gga = self.ddb.gga
                self.journal.ecrire("fix", {
                    "phase": phase,
                    "lat": lat, "lon": lon, "x": x, "y": y,
                    "dans_zone": bool(self.env.lim(x, y)),
                    "d_droite": d_droite, "d_gauche": d_gauche,
                    "dist_base": math.hypot(x, y),
                    "dist_d1": float(np.linalg.norm(np.array([x, y]) - self.d1)),
                    "qualite": gga[0] if gga else None,
                    "satellites": gga[1] if gga else None,
                    "hdop": gga[2] if gga else None,
                    "rejetees": self.ddb.trames_invalides,
                    "trame": self.ddb.derniere_trame_gll,
                })
            except Exception as exc:
                self.journal.ecrire("erreur_thread_gps", {
                    "erreur": repr(exc), "trace": traceback.format_exc()})
                time.sleep(0.5)

    # ------------------------------------------------------------------
    # Phases
    # ------------------------------------------------------------------
    def _position_fraiche(self):
        position = self.ddb.get_DDboat_cart_position(
            self.env.home_latlong, self.env.rotation)
        age = self.ddb.age_fix_gps()
        if position is None or age is None or age > AGE_FIX_MAX:
            return None, age
        return np.array(position, dtype=float), age

    def _attendre_gps(self):
        debut = time.monotonic()
        while time.monotonic() - debut < self.options["timeout_gps"]:
            position, _ = self._position_fraiche()
            if position is not None:
                return position
            time.sleep(0.2)
        return None

    def _compte_a_rebours(self):
        fin = time.monotonic() + self.options["attente"]
        while True:
            reste = fin - time.monotonic()
            if reste <= 0:
                return
            cap = self.ddb.current_cap_rad()
            position, age = self._position_fraiche()
            self.definir_message("depart dans {:.0f} s".format(math.ceil(reste)))
            # Le bateau est immobile : un cap qui varie ici signale un
            # probleme de magnetometre ou de calibration.
            self.journal.ecrire("attente", {
                "cap": math.degrees(cap), "mag": self.ddb.dernier_mag_brut,
                "x": position[0] if position is not None else None,
                "y": position[1] if position is not None else None,
                "age": age})
            time.sleep(0.5)

    def _aller_d1(self, depart):
        """Retourne "sortie", ou la raison de l'abandon."""
        opts = self.options
        debut = time.monotonic()
        hors_consecutifs = 0
        dernier_fix = self.ddb.compteur_fix_gps
        direction_finale = None  # cap d'approche garde pres de D1
        distance_finale = None   # distance a D1 au moment du passage
        gps_perime = False
        precedent = time.monotonic()
        try:
            while True:
                maintenant = time.monotonic()
                dt = maintenant - precedent
                precedent = maintenant
                if maintenant - debut > opts["timeout_aller"]:
                    return "timeout"

                position, age = self._position_fraiche()
                if position is None:
                    self.ddb._arreter_moteurs()
                    if not gps_perime:
                        self.journal.ecrire("gps_perime", {"age": age})
                        self.definir_message("GPS perdu, moteurs coupes")
                        gps_perime = True
                    time.sleep(0.2)
                    continue
                if gps_perime:
                    self.journal.ecrire("gps_retabli", {"age": age})
                    gps_perime = False

                x, y = position
                dist_base = dist_cart(position, self.env.home)
                vers_d1 = self.d1 - position
                dist_d1 = float(np.linalg.norm(vers_d1))
                if dist_base > opts["distance_max"]:
                    return "distance_max"

                # Pres de D1, la direction vers D1 devient instable (bruit
                # GPS) : on garde la direction d'approche jusqu'a la sortie.
                if direction_finale is None and dist_d1 <= opts["rayon_d1"]:
                    direction_finale = self.d1 - depart
                    direction_finale /= np.linalg.norm(direction_finale)
                    distance_finale = dist_d1
                    self.journal.ecrire("d1_atteint", {
                        "x": x, "y": y, "dist_d1": dist_d1})
                    self.definir_message("D1 atteint, franchissement")

                if direction_finale is not None:
                    direction = direction_finale
                    # Distance parcourue au-dela de D1 le long de l'approche.
                    depassement = float(np.dot(position - self.d1, direction_finale))
                    if depassement > opts["depassement_max"]:
                        return "sortie_non_detectee"
                else:
                    direction = vers_d1 / max(dist_d1, 1e-6)
                    depassement = None

                # Detection de sortie : uniquement sur de nouveaux fix.
                dans_zone = bool(self.env.lim(x, y))
                if self.ddb.compteur_fix_gps != dernier_fix:
                    dernier_fix = self.ddb.compteur_fix_gps
                    if dans_zone:
                        hors_consecutifs = 0
                    else:
                        hors_consecutifs += 1
                        self.journal.ecrire("hors_limites_fix", {
                            "x": x, "y": y, "compte": hors_consecutifs})
                        self.definir_message("hors limites {}/{}".format(
                            hors_consecutifs, opts["confirmations"]))
                        if hors_consecutifs >= opts["confirmations"]:
                            return "sortie"

                cible, cap, gauche, droite = self.ddb.maintien_cap(
                    direction, self.env.rotation)
                self.journal.ecrire("nav", {
                    "x": x, "y": y, "age": age, "dt": dt,
                    "dist_base": dist_base, "dist_d1": dist_d1,
                    "depassement": depassement, "dans_zone": dans_zone,
                    "direction": direction, "cap_cible": math.degrees(cible),
                    "cap": math.degrees(cap),
                    "erreur": math.degrees(angle_diff(cible, cap)),
                    "moteurs": (gauche, droite), "mag": self.ddb.dernier_mag_brut,
                })
                if direction_finale is None:
                    self.definir_message("D1 a {:.0f} m".format(dist_d1))
                time.sleep(0.05)
        finally:
            self.ddb._arreter_moteurs()
            if distance_finale is not None:
                self.journal.ecrire("aller_fin", {"dist_d1_passage": distance_finale})

    # ------------------------------------------------------------------
    # Programme principal
    # ------------------------------------------------------------------
    def _journaliser_configuration(self):
        env = self.env
        self.journal.ecrire("configuration", {
            "options": self.options,
            "commit": _commit_git(),
            "python": sys.version.split()[0],
            "machine": platform.node(),
            "home_latlong": env.home_latlong,
            "rotation": env.rotation,
            "couloir": env.couloir,
            "bordure_gauche": env.bordure_gauche,
            "bordure_droite": env.bordure_droite,
            "d1": self.d1,
            "SENS_ROTATION": DDBoat.SENS_ROTATION,
            "SENS_TOUR": DDBoat.SENS_TOUR,
            "VITESSE_ROTATION": DDBoat.VITESSE_ROTATION,
            "TOLERANCE_CAP_deg": math.degrees(DDBoat.TOLERANCE_CAP),
            "TIMEOUT_PALIER": DDBoat.TIMEOUT_PALIER,
        })

    def executer(self):
        """Execute la mission. Retourne 0 si la base est atteinte."""
        debut = time.monotonic()
        resultat = "inconnu"
        gps = None
        print("Journal : {}".format(self.journal.chemin))
        try:
            self._journaliser_configuration()

            from arduino_driver_v2 import ArduinoIO
            from gps_driver_v2 import GpsIO
            from imu9_driver_v2 import Imu9IO

            ard = ArduinoIO()
            gps = GpsIO(tty_dev=self.options["gps"])
            imu = Imu9IO()
            ddb = DDBoat(ard, gps, imu)
            ddb.journal = self.journal
            ddb._ensure_calibration()
            self.journal.ecrire("calibration", {"invA": ddb.invA, "xbar": ddb.xbar})
            self.ddb = ddb
            ddb._arreter_moteurs()
            ddb.current_cap_rad()

            self.thread_gps = threading.Thread(target=self._boucle_gps, name="gps")
            self.thread_gps.daemon = True
            self.thread_gps.start()

            self.definir_phase("ATTENTE_GPS", "en attente d'un fix GPS")
            depart = self._attendre_gps()
            if depart is None:
                resultat = "pas_de_gps"
                self.definir_phase("ECHEC", "aucun fix GPS")
                return 1

            distance_base = dist_cart(depart, self.env.home)
            self.journal.ecrire("depart", {"x": depart[0], "y": depart[1],
                                           "dist_base": distance_base,
                                           "dans_zone": bool(self.env.lim(*depart))})
            if distance_base > 15.0:
                print("Attention : le bateau est a {:.0f} m de la base.".format(distance_base))

            self.definir_phase("DEPART", "depart dans {:.0f} s".format(self.options["attente"]))
            self._compte_a_rebours()
            depart, _ = self._position_fraiche()
            if depart is None:
                depart = np.array(self.env.home, dtype=float)

            self.definir_phase("ALLER_D1", "D1 a {:.0f} m".format(
                float(np.linalg.norm(self.d1 - depart))))
            issue = self._aller_d1(depart)
            self.journal.ecrire("aller_issue", {"issue": issue})

            if issue == "sortie":
                self.definir_phase("HORS_LIMITES", "sortie de zone detectee")
                time.sleep(1.0)
                self.definir_phase("TOUR", "tour complet")
                tour_ok = ddb.tour_complet()
                self.journal.ecrire("tour_fin", {"reussi": bool(tour_ok)})
            else:
                print("Aller interrompu : {}".format(issue))

            self.definir_phase("RETOUR", "retour par le couloir" if issue == "sortie"
                               else "aller interrompu ({}), retour".format(issue))
            arrive = ddb.homing(self.env,
                                rayon_arrivee=self.options["rayon_arrivee"],
                                timeout=self.options["timeout_retour"],
                                age_fix_max=AGE_FIX_MAX)
            if arrive:
                resultat = "arrivee" if issue == "sortie" else "arrivee_sans_sortie"
                self.definir_phase("ARRIVEE", "base atteinte")
                return 0
            resultat = "retour_timeout"
            self.definir_phase("ECHEC", "retour non termine")
            return 1
        except BaseException as exc:
            # Ctrl+C (SystemExit via ArduinoIO), KeyboardInterrupt ou erreur.
            resultat = "interrompu" if isinstance(exc, (SystemExit, KeyboardInterrupt)) else "erreur"
            self.journal.ecrire("exception", {
                "erreur": repr(exc), "trace": traceback.format_exc()})
            self.definir_phase("ARRET" if resultat == "interrompu" else "ECHEC",
                               repr(exc))
            raise
        finally:
            if self.ddb is not None:
                try:
                    self.ddb._arreter_moteurs()
                except Exception:
                    pass
            self.stop_gps.set()
            if self.thread_gps is not None:
                self.thread_gps.join(timeout=1.0)
            position = self._position()
            self.journal.ecrire("fin", {
                "resultat": resultat,
                "duree": time.monotonic() - debut,
                "x": position[0] if position is not None else None,
                "y": position[1] if position is not None else None,
                "dist_base": math.hypot(position[0], position[1]) if position is not None else None,
            })
            self.journal.fermer()
            self.termine = True
            if gps is not None:
                try:
                    gps.close()
                except Exception:
                    pass
            print("Mission terminee ({}). Journal : {}".format(resultat, self.journal.chemin))
