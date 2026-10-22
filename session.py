# -*- coding: utf-8 -*-
"""Session du bateau : materiel, etat partage avec l'IHM, journal, missions.

Une session ouvre le materiel une seule fois (Arduino, GPS, compas) puis
execute les missions demandees, l'une apres l'autre, depuis le thread
principal : c'est la condition pour que Ctrl+C coupe les moteurs (gestionnaire
de ArduinoIO). Le serveur de l'IHM tourne dans un autre thread et ne fait que
lire l'etat (`etat()`) ou poser une demande (`lancer()`, `arreter()`).

Chaque mission ecrit son propre journal logs/<mission>_AAAAMMJJ_HHMMSS.jsonl :
une ligne JSON par evenement (configuration, phases, fix GPS, commandes de
cap, guidage, erreurs avec trace). Il est telechargeable depuis l'IHM
(/journal).
"""

import datetime
import json
import math
import os
import platform
import queue
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
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

from ddboatclass import DDBoat, ArretDemande, angle_diff, dist_cart  # noqa: E402
import communication  # noqa: E402  (client UDP du serveur du lac, fige)

AGE_FIX_MAX = 3.0  # au-dela, la position GPS est consideree perimee (s)

# Communication avec l'autre bateau (voir Session._boucle_communication).
COMM_DEFAUT = {"ip": "", "port": 6800, "mon_id": 3, "autre_id": 4}
PERIODE_ENVOI = 1.0     # envoi de notre position (s)
PERIODE_LECTURE = 0.2   # relecture de l'etat des bateaux (s)
AGE_LIEN_MAX = 3.0      # au-dela, le lien avec l'autre bateau est affiche perdu (s)
DIST_AUTRE_MAX = 200.0  # une position a plus de ... m de la base est rejetee


# ----------------------------------------------------------------------
# Journal
# ----------------------------------------------------------------------
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

    def __init__(self, dossier, prefixe="mission"):
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


# ----------------------------------------------------------------------
# Missions
# ----------------------------------------------------------------------
class Mission(object):
    """Classe de base d'une mission. Les sous-classes definissent :

    NOM, TITRE, DESCRIPTION : identification dans l'IHM ;
    PARAMETRES : liste de dicts {nom, label, defaut, type, aide} avec type
                 "nombre", "entier", "choix" (cle "choix") ou "texte" ;
    executer() : retourne (code, resultat), code 0 si tout s'est bien passe.
    """

    NOM = ""
    TITRE = ""
    DESCRIPTION = ""
    PARAMETRES = []

    def __init__(self, session, parametres):
        self.s = session
        self.p = parametres
        self.env = session.env

    @property
    def ddb(self):
        return self.s.ddb

    @property
    def journal(self):
        return self.s.journal

    @classmethod
    def apercu(cls, session, parametres):
        """Points [[x, y], ...] a afficher avant le lancement, ou None."""
        return None

    def executer(self):
        raise NotImplementedError


# ----------------------------------------------------------------------
# Session
# ----------------------------------------------------------------------
class Session(object):
    PHASES = {
        "VEILLE": "En attente",
        "INIT": "Initialisation",
        "ATTENTE_GPS": "Attente GPS",
        "DEPART": "Depart",
        "ALLER_D1": "Aller vers D1",
        "HORS_LIMITES": "Hors limites detecte",
        "TOUR": "Tour complet",
        "RETOUR": "Retour par le couloir",
        "ARRIVEE": "Base atteinte",
        "ESSAI": "Essai en cours",
        "NAVIGATION": "Navigation",
        "MAINTIEN": "Maintien de position",
        "RALLIEMENT": "Retour dans la zone",
        "ATTENTE_AUTRE": "Attente de l'autre bateau",
        "EVITEMENT": "Evitement de l'autre bateau",
        "BUT_OCCUPE": "But occupe : attente",
        "ARRET_URGENCE": "Arret d'urgence (autre bateau trop proche)",
        "LIEN_PERDU": "Lien perdu avec l'autre bateau",
        "TERMINE": "Mission terminee",
        "ECHEC": "Echec",
        "ARRET": "Arret",
    }

    def __init__(self, env, gps=0, simulation=None, dossier_logs=None,
                 fichier_comm=None):
        self.env = env
        self.gps_index = gps
        self.simulation = simulation
        if dossier_logs:
            self.dossier_logs = Path(dossier_logs)
        elif simulation is not None:
            # Les journaux simules ne se melangent jamais aux vrais.
            self.dossier_logs = BASE_DIR / "logs" / "simulation"
        else:
            self.dossier_logs = BASE_DIR / "logs"
        self.missions = {}
        self.verrou = threading.Lock()
        self.phase = "VEILLE"
        self.message = ""
        self.trail = deque(maxlen=1500)
        self.guidage = None
        self.ddb = None
        self.monde = None
        self.journal = None
        self.chemin_journal = None
        self.en_cours = False
        self.info_mission = self._info_vide()
        self._file = queue.Queue()
        self._stop_materiel = False
        self._thread_materiel = None
        self._gps = None
        self.convertisseur = DDBoat(None, None, None)
        # Communication avec l'autre bateau : etat partage entre le thread de
        # communication, les missions et l'IHM, protege par _verrou_comm.
        self.fichier_comm = (Path(fichier_comm) if fichier_comm
                             else BASE_DIR / "communication.json")
        self._verrou_comm = threading.Lock()
        self._config_comm = self._lire_config_communication()
        self._autre = None
        self._erreur_comm = None
        self._derniere_erreur_comm = None
        self._thread_comm = None

    @staticmethod
    def _info_vide():
        return {"nom": None, "titre": None, "etat": "veille", "resultat": None,
                "parametres": {}, "debut": None, "duree": None}

    # ------------------------------------------------------------------
    # Missions disponibles
    # ------------------------------------------------------------------
    def enregistrer(self, classe):
        self.missions[classe.NOM] = classe

    def description_missions(self):
        liste = []
        for classe in self.missions.values():
            liste.append({"nom": classe.NOM, "titre": classe.TITRE,
                          "description": classe.DESCRIPTION,
                          "parametres": classe.PARAMETRES})
        return liste

    @staticmethod
    def valider_parametres(classe, brut):
        """Convertit les valeurs recues (texte) selon la declaration."""
        brut = brut or {}
        valeurs = {}
        for definition in classe.PARAMETRES:
            nom = definition["nom"]
            valeur = brut.get(nom, definition["defaut"])
            genre = definition.get("type", "nombre")
            try:
                if genre == "nombre":
                    valeur = float(valeur)
                    if not math.isfinite(valeur):
                        raise ValueError("valeur non finie")
                elif genre == "entier":
                    valeur = int(float(valeur))
                elif genre == "choix":
                    valeur = str(valeur)
                    if valeur not in definition["choix"]:
                        raise ValueError("choix inconnu")
                else:
                    valeur = str(valeur).strip()
            except (TypeError, ValueError):
                raise ValueError("parametre invalide : {}".format(definition["label"]))
            valeurs[nom] = valeur
        return valeurs

    def apercu(self, nom, brut):
        classe = self.missions.get(nom)
        if classe is None:
            return None
        try:
            parametres = self.valider_parametres(classe, brut)
            points = classe.apercu(self, parametres)
        except Exception:
            return None
        if not points:
            return None
        return [[round(float(p[0]), 2), round(float(p[1]), 2)] for p in points]

    # ------------------------------------------------------------------
    # Materiel
    # ------------------------------------------------------------------
    def ouvrir_materiel(self):
        if self.simulation is not None:
            import simulation
            parametres = dict(self.simulation)
            ard, gps, imu, self.monde = simulation.creer_materiel(
                self.env.home_latlong, self.env.rotation, **parametres)
        else:
            from arduino_driver_v2 import ArduinoIO
            from gps_driver_v2 import GpsIO
            from imu9_driver_v2 import Imu9IO
            ard = ArduinoIO()
            gps = GpsIO(tty_dev=self.gps_index)
            imu = Imu9IO()
        self._gps = gps
        ddb = DDBoat(ard, gps, imu)
        if self.simulation is not None:
            ddb.invA = np.eye(3)
            ddb.xbar = np.zeros(3)
        else:
            ddb._ensure_calibration()
        ddb._arreter_moteurs()
        self.ddb = ddb
        self._stop_materiel = False
        self._thread_materiel = threading.Thread(
            target=self._boucle_materiel, name="materiel")
        self._thread_materiel.daemon = True
        self._thread_materiel.start()
        # La communication tourne des le demarrage, en veille comme en mission.
        self._thread_comm = threading.Thread(
            target=self._boucle_communication, name="communication")
        self._thread_comm.daemon = True
        self._thread_comm.start()

    def fermer_materiel(self):
        self._stop_materiel = True
        if self._thread_materiel is not None:
            self._thread_materiel.join(timeout=1.0)
        if self._thread_comm is not None:
            self._thread_comm.join(timeout=1.0)
        if self.ddb is not None:
            try:
                self.ddb._arreter_moteurs()
            except Exception:
                pass
        if self._gps is not None:
            try:
                self._gps.close()
            except Exception:
                pass

    def _boucle_materiel(self):
        """Lecture continue du GPS (et du compas en veille), meme pendant un tour."""
        dernier_compteur = 0
        dernier_cap = 0.0
        while not self._stop_materiel:
            try:
                self.ddb.mise_a_jour_gps()
                if not self.en_cours and time.monotonic() - dernier_cap > 0.2:
                    dernier_cap = time.monotonic()
                    self.ddb.current_cap_rad()
                if self.ddb.compteur_fix_gps != dernier_compteur:
                    dernier_compteur = self.ddb.compteur_fix_gps
                    self._enregistrer_fix()
            except Exception as exc:
                self.ecrire_journal("erreur_thread_materiel", {
                    "erreur": repr(exc), "trace": traceback.format_exc()})
                time.sleep(0.5)
            time.sleep(0.1)

    def _enregistrer_fix(self):
        position = self._position_locale()
        if position is None:
            return
        x, y = position
        with self.verrou:
            if (not self.trail or math.hypot(
                    x - self.trail[-1][0], y - self.trail[-1][1]) >= 0.7):
                self.trail.append([round(x, 1), round(y, 1)])
            phase = self.phase
        if self.journal is None:
            return
        ddb = self.ddb
        d_droite, d_gauche = distances_bordures(self.env, x, y)
        lat, lon = ddb.derniere_position_gps
        gga = ddb.gga
        donnees = {
            "phase": phase, "lat": lat, "lon": lon, "x": x, "y": y,
            "dans_zone": bool(self.env.lim(x, y)),
            "d_droite": d_droite, "d_gauche": d_gauche,
            "dist_base": math.hypot(x, y),
            "qualite": gga[0] if gga else None,
            "satellites": gga[1] if gga else None,
            "hdop": gga[2] if gga else None,
            "rejetees": ddb.trames_invalides,
            "trame": ddb.derniere_trame_gll,
        }
        sol = ddb.vitesse_sol
        if sol:
            donnees.update({"v_sol": sol["vitesse"], "cog": sol["cog"],
                            "cap_moyen": sol.get("cap_moyen"),
                            "commande_moy": sol.get("commande")})
        estimation = ddb.derive.estimer(time.monotonic())
        if estimation:
            donnees.update({"derive_est": estimation["est"],
                            "derive_nord": estimation["nord"],
                            "s_eau": estimation["s"],
                            "derive_n": estimation["n"],
                            "derive_rms": estimation["rms"]})
        self.journal.ecrire("fix", donnees)

    def ecrire_journal(self, type_evenement, donnees=None):
        journal = self.journal
        if journal is not None:
            journal.ecrire(type_evenement, donnees)

    # ------------------------------------------------------------------
    # Communication avec l'autre bateau
    # ------------------------------------------------------------------
    @staticmethod
    def valider_config_communication(brut):
        """Configuration {ip, port, mon_id, autre_id} validee, ou ValueError."""
        brut = brut or {}
        ip = str(brut.get("ip", COMM_DEFAUT["ip"])).strip()
        if len(ip) > 253 or any(c.isspace() for c in ip):
            raise ValueError("adresse du serveur invalide")
        try:
            port = int(float(brut.get("port", COMM_DEFAUT["port"])))
            mon_id = int(float(brut.get("mon_id", COMM_DEFAUT["mon_id"])))
            autre_id = int(float(brut.get("autre_id", COMM_DEFAUT["autre_id"])))
        except (TypeError, ValueError):
            raise ValueError("port et identifiants : nombres entiers attendus")
        if not 1 <= port <= 65535:
            raise ValueError("port invalide (1 a 65535)")
        if mon_id < 0 or autre_id < 0:
            raise ValueError("identifiant invalide")
        if mon_id == autre_id:
            raise ValueError("mon identifiant et celui de l'autre bateau sont identiques")
        return {"ip": ip, "port": port, "mon_id": mon_id, "autre_id": autre_id}

    def _lire_config_communication(self):
        try:
            with open(str(self.fichier_comm), encoding="utf-8") as fichier:
                return self.valider_config_communication(json.load(fichier))
        except FileNotFoundError:
            pass
        except (OSError, ValueError) as exc:
            print("Configuration de communication ignoree ({}) : {}".format(
                self.fichier_comm.name, exc))
        return dict(COMM_DEFAUT)

    def configuration_communication(self):
        with self._verrou_comm:
            return dict(self._config_comm)

    def configurer_communication(self, brut):
        """Applique et enregistre la configuration, sans redemarrer le programme."""
        config = self.valider_config_communication(brut)
        if self.en_cours:
            raise RuntimeError("une mission est en cours : reglages verrouilles")
        with self._verrou_comm:
            ancienne = self._config_comm
            self._config_comm = config
            if (config["ip"], config["autre_id"]) != (ancienne["ip"], ancienne["autre_id"]):
                self._autre = None
            self._erreur_comm = None
            self._derniere_erreur_comm = None
        try:
            temporaire = self.fichier_comm.with_name(self.fichier_comm.name + ".tmp")
            with open(str(temporaire), "w", encoding="utf-8") as fichier:
                json.dump(config, fichier, indent=2)
            os.replace(str(temporaire), str(self.fichier_comm))
        except OSError as exc:
            print("Configuration de communication non enregistree : {}".format(exc))
        return config

    def _boucle_communication(self):
        """Envoie notre position (1 Hz) et relit celle de l'autre bateau.

        Elle ne bloque jamais une mission : toute erreur est notee et la
        boucle continue. Le client communication.py est utilise tel quel.
        """
        dernier_envoi = None
        while not self._stop_materiel:
            with self._verrou_comm:
                config = dict(self._config_comm)
            if config["ip"]:
                maintenant = time.monotonic()
                if dernier_envoi is None or maintenant - dernier_envoi >= PERIODE_ENVOI:
                    dernier_envoi = maintenant
                    try:
                        self._envoyer_position(config)
                    except Exception as exc:
                        self._signaler_erreur_comm("envoi", exc)
                try:
                    self._lire_autre(config, maintenant)
                except Exception as exc:
                    self._signaler_erreur_comm("lecture", exc)
            time.sleep(PERIODE_LECTURE)

    def _envoyer_position(self, config):
        ddb = self.ddb
        if ddb is None or ddb.derniere_position_gps is None:
            return
        age = ddb.age_fix_gps()
        if age is None or age > AGE_FIX_MAX:
            # Un fix perime ne part pas : l'autre bateau verrait l'age du
            # paquet, pas celui du fix, et se fierait a une position fausse.
            return
        lat, lon = ddb.derniere_position_gps
        cap = ddb.dernier_cap
        communication.send_position(
            config["ip"], config["port"], config["mon_id"], lat, lon,
            math.degrees(cap) if cap is not None else 0.0)
        with self._verrou_comm:
            self._erreur_comm = None

    def _lire_autre(self, config, maintenant):
        """Filtre get_boats() : seul l'autre bateau, avec des valeurs plausibles."""
        bateau = communication.get_boats().get(config["autre_id"])
        if bateau is None:
            return
        try:
            lat = float(bateau["lat"])
            lon = float(bateau["lon"])
            cap = float(bateau["cap"])
            age = float(bateau["age"])
        except (KeyError, TypeError, ValueError):
            return
        if not all(math.isfinite(v) for v in (lat, lon, cap, age)):
            return
        if abs(lat) > 90.0 or abs(lon) > 180.0 or not 0.0 <= cap <= 360.0:
            return
        x, y = self.convertisseur.convert_cart_position(
            lat, lon, self.env.home_latlong, self.env.rotation)
        if math.hypot(x, y) > DIST_AUTRE_MAX:
            return
        with self._verrou_comm:
            self._autre = {"id": config["autre_id"], "lat": lat, "lon": lon,
                           "cap": cap % 360.0, "x": float(x), "y": float(y),
                           "age_lu": max(0.0, age), "t_lu": maintenant}

    def _signaler_erreur_comm(self, etape, exc):
        texte = "{} : {}".format(etape, exc)
        with self._verrou_comm:
            self._erreur_comm = texte
            nouvelle = texte != self._derniere_erreur_comm
            self._derniere_erreur_comm = texte
        if nouvelle:
            self.ecrire_journal("erreur_communication", {"erreur": texte})

    def etat_autre(self):
        """Dernier etat valide de l'autre bateau, ou None s'il n'a jamais ete vu.

        C'est une copie ; "age" est l'age du paquet recu plus le temps ecoule
        depuis notre lecture. Ce n'est pas l'age du fix GPS de l'autre bateau.
        """
        with self._verrou_comm:
            base = self._autre
            if base is None:
                return None
            copie = dict(base)
        copie["age"] = copie["age_lu"] + max(0.0, time.monotonic() - copie["t_lu"])
        return copie

    def _etat_autre_ihm(self):
        """Etat de l'autre bateau pour l'IHM : toujours un dict, avec le lien."""
        with self._verrou_comm:
            config = dict(self._config_comm)
            erreur = self._erreur_comm
        autre = self.etat_autre()
        etat = {"id": config["autre_id"], "lien": "ok", "age": None, "x": None,
                "y": None, "lat": None, "lon": None, "cap": None,
                "heading": None, "erreur": erreur}
        if not config["ip"]:
            etat["lien"] = "non_configure"
        elif autre is None:
            etat["lien"] = "aucune_donnee"
        else:
            etat.update({
                "age": autre["age"], "x": autre["x"], "y": autre["y"],
                "lat": autre["lat"], "lon": autre["lon"], "cap": autre["cap"],
                "heading": (math.pi / 2 - math.radians(autre["cap"])
                            - self.env.rotation),
            })
            if autre["age"] > AGE_LIEN_MAX:
                etat["lien"] = "perdu"
        return etat

    # ------------------------------------------------------------------
    # Etat partage avec l'IHM
    # ------------------------------------------------------------------
    def definir_phase(self, phase, message=""):
        with self.verrou:
            self.phase = phase
            self.message = message
        print("[{}] {}".format(phase, message))
        self.ecrire_journal("phase", {"phase": phase, "message": message})

    def definir_message(self, message):
        with self.verrou:
            self.message = message

    def definir_guidage(self, **champs):
        """Met a jour les informations de guidage affichees par l'IHM."""
        with self.verrou:
            if not champs:
                self.guidage = None
            else:
                if self.guidage is None:
                    self.guidage = {}
                self.guidage.update(champs)

    def _position_locale(self):
        """Derniere position locale (x, y) sans lire le materiel."""
        ddb = self.ddb
        if ddb is None or ddb.derniere_position_gps is None:
            return None
        lat, lon = ddb.derniere_position_gps
        return ddb.convert_cart_position(
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

    def chemin_journal_courant(self):
        return self.chemin_journal

    def _nom_journal(self):
        """Chemin du journal affiche par l'IHM (relatif au depot s'il y est)."""
        if not self.chemin_journal:
            return None
        try:
            return str(self.chemin_journal.relative_to(BASE_DIR))
        except ValueError:
            return str(self.chemin_journal)

    def etat(self):
        """Etat courant pour l'IHM (appele depuis le thread du serveur)."""
        with self.verrou:
            phase = self.phase
            message = self.message
            trail = list(self.trail)
            guidage = dict(self.guidage) if self.guidage else None
            info = dict(self.info_mission)
        ddb = self.ddb
        cap = ddb.dernier_cap if ddb is not None else None
        cap_cible = ddb.dernier_cap_cible if ddb is not None and self.en_cours else None
        etat = {
            "phase": phase,
            "phase_texte": self.PHASES.get(phase, phase),
            "message": message,
            "x": None, "y": None, "lat": None, "lon": None,
            "fix_age": ddb.age_fix_gps() if ddb is not None else None,
            "dans_zone": None,
            "cap": math.degrees(cap) if cap is not None else None,
            "cap_cible": math.degrees(cap_cible) if cap_cible is not None else None,
            # Angle mathematique (0 = axe x, sens anti-horaire) pour le dessin.
            "heading": (math.pi / 2 - cap - self.env.rotation) if cap is not None else None,
            "moteurs": self._moteurs(),
            "trail": trail,
            "journal": self._nom_journal(),
            "mission": info,
            "guidage": guidage,
            "derive": None,
            "vitesse_sol": None,
            "autre": self._etat_autre_ihm(),
        }
        if ddb is not None:
            estimation = ddb.derive.estimer(time.monotonic())
            if estimation:
                etat["derive"] = {
                    "vitesse": math.hypot(estimation["est"], estimation["nord"]),
                    "cap": math.degrees(math.atan2(estimation["est"], estimation["nord"])) % 360.0,
                    "n": estimation["n"],
                }
            if ddb.vitesse_sol:
                etat["vitesse_sol"] = ddb.vitesse_sol["vitesse"]
        position = self._position_locale()
        if position is not None:
            x, y = float(position[0]), float(position[1])
            lat, lon = ddb.derniere_position_gps
            etat.update({"x": x, "y": y, "lat": lat, "lon": lon,
                         "dans_zone": bool(self.env.lim(x, y))})
        return etat

    def position_actuelle(self):
        """(x, y, cap_deg) pour l'apercu des essais, ou None."""
        position = self._position_locale()
        if position is None:
            return None
        cap = self.ddb.dernier_cap
        return (float(position[0]), float(position[1]),
                math.degrees(cap) if cap is not None else None)

    # ------------------------------------------------------------------
    # Primitives pour les missions
    # ------------------------------------------------------------------
    def verifier_arret(self):
        if self.ddb.arret.is_set():
            raise ArretDemande()

    def pause(self, secondes):
        """Attente qui reagit a l'arret demande."""
        fin = time.monotonic() + secondes
        while time.monotonic() < fin:
            self.verifier_arret()
            time.sleep(min(0.05, max(0.0, fin - time.monotonic())))

    def position_fraiche(self):
        """((x, y) ou None, age du fix) ; None si le fix a plus de AGE_FIX_MAX s."""
        position = self.ddb.get_DDboat_cart_position(
            self.env.home_latlong, self.env.rotation)
        age = self.ddb.age_fix_gps()
        if position is None or age is None or age > AGE_FIX_MAX:
            return None, age
        return np.array(position, dtype=float), age

    def attendre_gps(self, timeout=120.0):
        debut = time.monotonic()
        while time.monotonic() - debut < timeout:
            self.verifier_arret()
            position, _ = self.position_fraiche()
            if position is not None:
                return position
            time.sleep(0.2)
        return None

    def compte_a_rebours(self, secondes):
        """Compte a rebours, bateau immobile : le cap doit rester stable."""
        fin = time.monotonic() + secondes
        while True:
            self.verifier_arret()
            reste = fin - time.monotonic()
            if reste <= 0:
                return
            cap = self.ddb.current_cap_rad()
            position, age = self.position_fraiche()
            self.definir_message("depart dans {:.0f} s".format(math.ceil(reste)))
            self.ecrire_journal("attente", {
                "cap": math.degrees(cap), "mag": self.ddb.dernier_mag_brut,
                "x": position[0] if position is not None else None,
                "y": position[1] if position is not None else None,
                "age": age})
            time.sleep(0.5)

    def suivre_chemin(self, suiveur, derive=True, vitesse=None, ecart_max=60.0,
                      timeout=900.0, etiquette="chemin", ralentir_fin=0.0,
                      vitesse_fin=90):
        """Suit un SuiveurChemin jusqu'a son dernier sommet.

        Sur le dernier segment, la vitesse diminue lineairement jusqu'a
        vitesse_fin sur les ralentir_fin metres qui precedent le rayon
        d'arret (0 = pas de ralentissement) : le bateau arrive doucement.

        Retourne (issue, stats) ; issue vaut "termine", "timeout" ou
        "ecart_max". stats contient, par segment parcouru, l'ecart lateral
        au segment (moyenne, ecart absolu moyen, rms, maximum).
        """
        ddb = self.ddb
        debut = time.monotonic()
        precedent = debut
        ddb.reinitialiser_asservissement()
        gps_perime = False
        segment_stats = self._nouvelles_stats(suiveur.segment)
        resume = []
        self.definir_guidage(
            chemin=[[float(p[0]), float(p[1])] for p in suiveur.points],
            n_segments=suiveur.nombre_segments, derive=bool(derive))
        issue = "termine"
        try:
            while True:
                self.verifier_arret()
                maintenant = time.monotonic()
                dt = maintenant - precedent
                precedent = maintenant
                if maintenant - debut > timeout:
                    issue = "timeout"
                    break
                position, age = self.position_fraiche()
                if position is None:
                    ddb._arreter_moteurs()
                    if not gps_perime:
                        self.ecrire_journal("gps_perime", {"age": age})
                        self.definir_message("GPS perdu, moteurs coupes")
                        gps_perime = True
                    time.sleep(0.2)
                    continue
                if gps_perime:
                    self.ecrire_journal("gps_retabli", {"age": age})
                    gps_perime = False

                segment_avant = suiveur.segment
                info = suiveur.evaluer(position)
                if info["sommet_atteint"] is not None:
                    self.ecrire_journal("sommet_atteint", {
                        "sommet": info["sommet_atteint"],
                        "x": position[0], "y": position[1],
                        "dist": float(np.linalg.norm(
                            position - suiveur.points[info["sommet_atteint"]]))})
                    resume.append(self._cloturer_stats(segment_stats, segment_avant,
                                                       maintenant, derive, etiquette))
                    segment_stats = self._nouvelles_stats(suiveur.segment, maintenant)
                if info["termine"]:
                    break
                ecart = suiveur.ecart_au_chemin(position)
                if ecart > ecart_max:
                    issue = "ecart_max"
                    self.ecrire_journal("ecart_max", {
                        "x": position[0], "y": position[1], "ecart": ecart})
                    break

                vitesse_ligne = vitesse
                if ralentir_fin > 0.0 and suiveur.sur_dernier_segment():
                    pleine = vitesse if vitesse is not None else ddb.vitesse_croisiere
                    # Distance qui reste avant le rayon d'arret.
                    restant = max(0.0, info["dist_cible"] - suiveur.rayon_final)
                    vitesse_ligne = vitesse_fin + (pleine - vitesse_fin) * min(
                        1.0, restant / ralentir_fin)
                    vitesse_ligne = min(pleine, vitesse_ligne)
                cible, cap, gauche, droite = ddb.maintien_cap(
                    info["direction"], self.env.rotation, derive=derive,
                    vitesse=vitesse_ligne)
                asserv = ddb.dernier_asservissement
                sol = ddb.vitesse_sol or {}
                if info["s"] >= 0.2 * info["longueur"]:
                    # Regime etabli : on ignore le demi-tour du depart.
                    self._accumuler_stats(segment_stats, info["lateral"])
                self.ecrire_journal("nav", dict({
                    "mission": etiquette,
                    "x": position[0], "y": position[1], "age": age, "dt": dt,
                    "segment": info["segment"], "s": info["s"],
                    "longueur": info["longueur"], "lateral": info["lateral"],
                    "dist_cible": info["dist_cible"], "ecart_chemin": ecart,
                    "direction": info["direction"],
                    "cap_cible": math.degrees(cible), "cap": math.degrees(cap),
                    "erreur": math.degrees(angle_diff(cible, cap)),
                    "moteurs": (gauche, droite), "mag": ddb.dernier_mag_brut,
                    "v_sol": sol.get("vitesse"), "cog": sol.get("cog"),
                }, **asserv))
                direction = np.asarray(info["direction"], dtype=float)
                norme = float(np.linalg.norm(direction))
                self.definir_guidage(
                    a=[float(info["a"][0]), float(info["a"][1])],
                    b=[float(info["b"][0]), float(info["b"][1])],
                    segment=int(info["segment"]),
                    lateral=float(info["lateral"]),
                    dist_cible=float(info["dist_cible"]),
                    direction=[float(direction[0] / norme), float(direction[1] / norme)]
                    if norme > 1e-9 else [0.0, 0.0],
                    route=asserv.get("route"), crabe=asserv.get("crabe"))
                time.sleep(0.05)
        finally:
            ddb._arreter_moteurs()
            if segment_stats["n"] > 0:
                resume.append(self._cloturer_stats(
                    segment_stats, suiveur.segment, time.monotonic(), derive,
                    etiquette, interrompu=True))
            self.definir_guidage()
        return issue, resume

    @staticmethod
    def _nouvelles_stats(segment, instant=None):
        return {"n": 0, "somme": 0.0, "somme_abs": 0.0, "somme_carres": 0.0,
                "max_abs": 0.0, "debut": instant if instant is not None else time.monotonic()}

    @staticmethod
    def _accumuler_stats(stats, lateral):
        stats["n"] += 1
        stats["somme"] += lateral
        stats["somme_abs"] += abs(lateral)
        stats["somme_carres"] += lateral * lateral
        stats["max_abs"] = max(stats["max_abs"], abs(lateral))

    def _cloturer_stats(self, stats, segment, instant, derive, etiquette,
                        interrompu=False):
        n = max(1, stats["n"])
        resume = {
            "mission": etiquette, "segment": segment, "derive": bool(derive),
            "n": stats["n"], "duree": instant - stats["debut"],
            "lateral_moy": stats["somme"] / n,
            "lateral_abs_moy": stats["somme_abs"] / n,
            "lateral_rms": math.sqrt(stats["somme_carres"] / n),
            "lateral_max": stats["max_abs"], "interrompu": interrompu,
        }
        self.ecrire_journal("segment_fin", resume)
        return resume

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------
    def lancer(self, nom, brut=None):
        """Demande une mission (appelable depuis un autre thread)."""
        classe = self.missions.get(nom)
        if classe is None:
            raise ValueError("mission inconnue : {}".format(nom))
        if self.ddb is None:
            raise RuntimeError("materiel non initialise")
        parametres = self.valider_parametres(classe, brut)
        with self.verrou:
            if self.en_cours or not self._file.empty():
                raise RuntimeError("une mission est deja en cours")
            self.info_mission = {"nom": nom, "titre": classe.TITRE,
                                 "etat": "demandee", "resultat": None,
                                 "parametres": parametres, "debut": None,
                                 "duree": None}
            self._file.put((nom, parametres))

    def arreter(self):
        """Arrete la mission en cours et coupe les moteurs (thread quelconque)."""
        if self.ddb is None:
            return
        self.ddb.arret.set()
        try:
            self.ddb.ard.send_arduino_cmd_motor(0, 0)
        except Exception:
            pass
        with self.verrou:
            # Mission demandee mais pas encore commencee : on l'annule.
            if not self.en_cours and not self._file.empty():
                try:
                    self._file.get_nowait()
                except queue.Empty:
                    pass
                self.info_mission.update({"etat": "arretee", "resultat": "annulee"})
                self.ddb.arret.clear()

    def boucle_principale(self):
        """Attend les demandes de l'IHM ; a executer dans le thread principal."""
        while True:
            try:
                nom, parametres = self._file.get(timeout=0.5)
            except queue.Empty:
                continue
            self.executer(nom, parametres)

    def _journaliser_configuration(self, classe, parametres):
        env = self.env
        ddb = self.ddb
        self.ecrire_journal("configuration", {
            "mission": classe.NOM,
            "parametres": parametres,
            "commit": _commit_git(),
            "python": sys.version.split()[0],
            "machine": platform.node(),
            "simulation": self.simulation is not None,
            "home_latlong": env.home_latlong,
            "rotation": env.rotation,
            "couloir": env.couloir,
            "bordure_gauche": env.bordure_gauche,
            "bordure_droite": env.bordure_droite,
            "SENS_ROTATION": DDBoat.SENS_ROTATION,
            "SENS_TOUR": DDBoat.SENS_TOUR,
            "VITESSE_ROTATION": DDBoat.VITESSE_ROTATION,
            "TOLERANCE_CAP_deg": math.degrees(DDBoat.TOLERANCE_CAP),
            "TIMEOUT_PALIER": DDBoat.TIMEOUT_PALIER,
            "regulateur": {
                "KP": DDBoat.KP_CAP, "KI": DDBoat.KI_CAP, "KD": DDBoat.KD_CAP,
                "limite_integrale": DDBoat.LIMITE_INTEGRALE,
                "vitesse": ddb.vitesse_croisiere,
                "compensation_derive": DDBoat.COMPENSATION_DERIVE,
            },
            "derive": {
                "vitesse_nominale": ddb.derive.VITESSE_NOMINALE,
                "fenetre": ddb.derive.FENETRE,
                "demi_vie": ddb.derive.DEMI_VIE,
            },
        })
        self.ecrire_journal("calibration", {"invA": ddb.invA, "xbar": ddb.xbar})

    def executer(self, nom, parametres):
        """Execute une mission dans le thread courant. Retourne le code (0 = ok)."""
        classe = self.missions[nom]
        ddb = self.ddb
        debut = time.monotonic()
        resultat = "inconnu"
        code = 1
        arret_programme = None
        with self.verrou:
            self.en_cours = True
            self.trail.clear()
            self.info_mission = {"nom": nom, "titre": classe.TITRE,
                                 "etat": "en_cours", "resultat": None,
                                 "parametres": parametres,
                                 "debut": time.time(), "duree": None}
        ddb.arret.clear()
        ddb.journal = None
        self.journal = Journal(self.dossier_logs, prefixe=nom)
        self.chemin_journal = self.journal.chemin
        ddb.journal = self.journal
        print("Journal : {}".format(self.journal.chemin))
        etat_final = "terminee"
        try:
            self._journaliser_configuration(classe, parametres)
            self.definir_phase("INIT", classe.TITRE)
            ddb._arreter_moteurs()
            ddb.current_cap_rad()
            mission = classe(self, parametres)
            code, resultat = mission.executer()
            if code != 0:
                etat_final = "echec"
        except ArretDemande:
            resultat = "arret_demande"
            code = 2
            etat_final = "arretee"
            self.ecrire_journal("arret_demande", {})
            self.definir_phase("ARRET", "arret demande")
        except BaseException as exc:
            interrompu = isinstance(exc, (SystemExit, KeyboardInterrupt))
            resultat = "interrompu" if interrompu else "erreur"
            etat_final = "arretee" if interrompu else "echec"
            self.ecrire_journal("exception", {
                "erreur": repr(exc), "trace": traceback.format_exc()})
            self.definir_phase("ARRET" if interrompu else "ECHEC", repr(exc))
            if interrompu:
                arret_programme = exc
            else:
                traceback.print_exc()
        finally:
            try:
                ddb._arreter_moteurs()
            except Exception:
                pass
            position = self._position_locale()
            duree = time.monotonic() - debut
            self.ecrire_journal("fin", {
                "resultat": resultat, "duree": duree,
                "x": position[0] if position is not None else None,
                "y": position[1] if position is not None else None,
                "dist_base": math.hypot(position[0], position[1]) if position is not None else None,
            })
            self.journal.fermer()
            self.journal = None
            ddb.journal = None
            ddb.arret.clear()
            self.definir_guidage()
            with self.verrou:
                self.en_cours = False
                self.info_mission.update({"etat": etat_final, "resultat": resultat,
                                          "duree": duree})
            print("Mission terminee ({}). Journal : {}".format(
                resultat, self.chemin_journal))
        if arret_programme is not None:
            raise arret_programme
        return code
