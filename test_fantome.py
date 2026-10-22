#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mission ABBA contre un bateau fantome, sans materiel.

    python3 test_fantome.py                    # les quatre scenarios
    python3 test_fantome.py --scenario face    # un seul
    python3 test_fantome.py --graines 1,2,3,4,5

Le bateau est simule (simulation.py, vent du lac). communication.py est fige :
le fantome est injecte en remplacant communication.send_position (sans effet)
et communication.get_boats (position scriptee, rafraichie a 1 Hz comme les
paquets du serveur, avec l'age du paquet). Dans face et croise, le fantome evite comme
nous (memes regles, cap limite) : l'autre bateau doit toujours eviter. Les
variantes face_passif et croise_passif (fantome qui fonce droit) sont donnees
a titre informatif : ce sont de vraies routes de collision, le resultat attendu
est une collision (meme sans repulsion).

Scenarios (A -> B, 91 m) :
  face      fantome de B vers A, en face-a-face ;
  croise    le fantome coupe le milieu du trajet a angle droit ;
  occupe    fantome immobile au but, il part 30 s apres que le bateau est
            passe a moins de 15 m du but ;
  lien      fantome loin du trajet, ses paquets s'arretent de 25 s a 45 s ;
  face_passif, croise_passif (informatifs) comme face et croise, mais le
            fantome suit une droite sans eviter.

Reglages d'essai : --graines 1,2,3 (bruit GPS/compas, donc cote d'evitement),
--param cle=valeur (parametre de la mission), --sans-evitement
(verifie que le scenario est bien une route de collision).

Sorties dans /tmp/opencode/logs_communication/<scenario>/ : journal JSONL de
la mission et trace.csv (positions vraies des deux bateaux). Les vrais
journaux logs/ ne sont jamais touches.
"""

import argparse
import json
import math
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

BASE_DIR = Path(__file__).resolve().parent
SORTIE = Path("/tmp/opencode/logs_communication")
VITESSE_FANTOME = 1.3     # m/s
SCENARIOS = ["face", "croise", "occupe", "lien"]
INFORMATIFS = ["face_passif", "croise_passif"]
ID_MOI, ID_FANTOME = 3, 4
PHASES_AVANT_DEPART = ("VEILLE", "INIT", "ATTENTE_GPS", "ATTENTE_AUTRE", "DEPART")
COUPURE = (25.0, 45.0)    # scenario lien : paquets perdus entre ces instants (s apres le depart)

sys.path.insert(0, str(BASE_DIR))
sys.path.insert(0, str(BASE_DIR / "drivers"))


def charger_points():
    with open(str(BASE_DIR / "points.json"), encoding="utf-8") as f:
        return {p["nom"]: p for p in json.load(f)["points"]}


class Fantome(object):
    """Autre bateau scripte : fournit get_boats() et send_position()."""

    def __init__(self, depart, trajectoire, local_vers_latlon, coupure=None):
        self.depart = np.array(depart, dtype=float)
        self.trajectoire = trajectoire    # (fantome, t) -> (x, y), t en s depuis le depart
        self.local_vers_latlon = local_vers_latlon
        self.coupure = coupure
        self.t_go = None                  # instant du depart (None = immobile)
        self.t_part = None                # scenario occupe : instant ou il part
        self.dernier_envoi = None         # ce que nous avons envoye (echo du serveur)
        self.dernier_cap = 0.0
        self.mon_etat = None

    def position(self, instant):
        if self.t_go is None:
            return self.depart.copy()
        return np.array(self.trajectoire(self, instant - self.t_go), dtype=float)

    def cap(self, instant):
        a = self.position(instant - 0.5)
        b = self.position(instant + 0.5)
        v = b - a
        if math.hypot(v[0], v[1]) > 0.2:
            self.dernier_cap = math.degrees(math.atan2(v[0], v[1])) % 360.0
        return self.dernier_cap

    def instant_paquet(self, instant):
        """Instant du dernier paquet recu : 1 Hz, avec une coupure eventuelle."""
        paquet = math.floor(instant)
        if self.coupure is not None and self.t_go is not None:
            debut = self.t_go + self.coupure[0]
            fin = self.t_go + self.coupure[1]
            if debut <= instant < fin:
                paquet = min(paquet, debut)
        return paquet

    # --- remplacent communication.* ---------------------------------------
    def send_position(self, host, port, boat_id, lat, lon, cap):
        self.mon_etat = (boat_id, lat, lon, cap)
        return "ok"

    def get_boats(self):
        instant = time.monotonic()
        paquet = self.instant_paquet(instant)
        x, y = self.position(paquet)
        lat, lon = self.local_vers_latlon(x, y)
        boats = {ID_FANTOME: {"lat": lat, "lon": lon, "cap": self.cap(paquet),
                              "age": instant - paquet}}
        if self.mon_etat is not None:
            ident, mlat, mlon, mcap = self.mon_etat
            boats[ident] = {"lat": mlat, "lon": mlon, "cap": mcap % 360.0, "age": 0.3}
        return boats


class FantomeReactif(Fantome):
    """Fantome qui evite comme nous : memes regles, cap limite a 25 deg/s."""

    VITESSE_ROTATION = math.radians(25.0)

    def __init__(self, depart, but, local_vers_latlon):
        Fantome.__init__(self, depart, None, local_vers_latlon)
        self.but = np.array(but, dtype=float)
        self.pos = np.array(depart, dtype=float)
        self.cap_rad = math.atan2(self.but[0] - self.pos[0], self.but[1] - self.pos[1])
        self.historique = [(0.0, self.pos[0], self.pos[1])]
        self.dernier_pas = None

    def position(self, instant):
        courant = self.historique[0]
        for echantillon in self.historique:
            if echantillon[0] > instant:
                break
            courant = echantillon
        return np.array([courant[1], courant[2]])

    def avancer(self, instant, autre):
        import mission_communication as mc
        if self.dernier_pas is None:
            self.dernier_pas = instant
        dt = max(0.0, min(0.5, instant - self.dernier_pas))
        self.dernier_pas = instant
        vers = self.but - self.pos
        d_but = float(np.linalg.norm(vers))
        d = float(np.linalg.norm(self.pos - autre))
        if d_but > 3.0 and d >= 5.0:
            rep, _ = mc.calculer_repulsion(self.pos, autre, 15.0)
            total = vers / d_but + rep
            if float(np.linalg.norm(total)) > 1e-3:
                voulu = math.atan2(total[0], total[1])
                ecart = (voulu - self.cap_rad + math.pi) % (2 * math.pi) - math.pi
                pas = max(-self.VITESSE_ROTATION * dt, min(self.VITESSE_ROTATION * dt, ecart))
                self.cap_rad += pas
                reduction = max(0.0, min(1.0, 1.0 - (abs(ecart) - math.radians(35.0))
                                         / math.radians(55.0)))
                vitesse = VITESSE_FANTOME * mc.facteur_vitesse(d, 5.0, 15.0) * reduction
                self.pos = self.pos + vitesse * dt * np.array(
                    [math.sin(self.cap_rad), math.cos(self.cap_rad)])
        self.historique.append((instant, self.pos[0], self.pos[1]))
        del self.historique[:-400]
        self.dernier_cap = math.degrees(self.cap_rad) % 360.0

    def cap(self, instant):
        return self.dernier_cap


def construire_scenario(nom, A, B):
    """Retourne (trajectoire, depart du fantome, coupure, but du fantome reactif)."""
    ab = B - A
    L = float(np.linalg.norm(ab))
    u = ab / L
    n = np.array([-u[1], u[0]])
    milieu = A + u * L / 2.0
    coupure = None
    but = None
    if nom in ("face", "face_passif"):
        def trajectoire(f, t):
            return B - u * min(VITESSE_FANTOME * t, L + 20.0)
        depart = B
        but = A
    elif nom in ("croise", "croise_passif"):
        # Au milieu en meme temps que le bateau (~37 s a 1,2 m/s).
        decalage = 48.0

        def trajectoire(f, t):
            return milieu + n * (decalage - VITESSE_FANTOME * t)
        depart = milieu + n * decalage
        but = milieu - n * decalage
    elif nom == "occupe":
        def trajectoire(f, t):
            if f.t_part is None or time.monotonic() < f.t_part:
                return B
            return B + u * VITESSE_FANTOME * (time.monotonic() - f.t_part)
        depart = B
    elif nom == "lien":
        def trajectoire(f, t):
            return B + n * 40.0 - u * min(VITESSE_FANTOME * t, L)
        depart = B + n * 40.0
        coupure = COUPURE
    else:
        raise ValueError("scenario inconnu : {}".format(nom))
    return trajectoire, depart, coupure, but


def lire_journal(dossier):
    fichiers = sorted(Path(dossier).glob("abba_*.jsonl"))
    if not fichiers:
        return []
    evenements = []
    with open(str(fichiers[-1]), encoding="utf-8") as f:
        for ligne in f:
            ligne = ligne.strip()
            if ligne:
                evenements.append(json.loads(ligne))
    return evenements


def executer_scenario(nom, acceleration, graine, sans_evitement=False,
                      supplementaires=None):
    import ihm_lac

    # Installe les doublures de smbus/serial avant d'importer les drivers.
    env = ihm_lac.charger_environnement()
    import communication
    import mission_communication
    from ddboatclass import DDBoat
    from missions import enregistrer_missions
    from session import Session

    points = charger_points()
    dossier = SORTIE / nom
    dossier.mkdir(parents=True, exist_ok=True)

    session = None
    arret = threading.Event()
    try:
        # Repere local : A (base) = origine.
        convertisseur = DDBoat(None, None, None)

        def vers_local(p):
            return np.array(convertisseur.convert_cart_position(
                p["lat"], p["lon"], env.home_latlong, env.rotation))

        A = vers_local(points["A"])
        B = vers_local(points["B"])
        cap_ab = math.degrees(math.atan2(B[0] - A[0], B[1] - A[1])) % 360.0
        trajectoire, depart_f, coupure, but_f = construire_scenario(nom, A, B)
        reactif = nom in ("face", "croise")
        def vers_latlon(x, y):
            return ihm_lac.local_vers_latlon(x, y, env.home_latlong, env.rotation)

        if reactif:
            fantome = FantomeReactif(depart_f, but_f, vers_latlon)
        else:
            fantome = Fantome(depart_f, trajectoire, vers_latlon, coupure)
        communication.send_position = fantome.send_position
        communication.get_boats = fantome.get_boats
        if sans_evitement:
            mission_communication.calculer_repulsion = (
                lambda position, autre, rayon, **kw: (
                    np.zeros(2), float(np.linalg.norm(
                        np.asarray(position) - np.asarray(autre)))))

        simulation = {"acceleration": acceleration, "vent": (0.0, 0.4),
                      "depart": (float(A[0]), float(A[1]), cap_ab),
                      "gain_rotation": 0.17, "tau_rotation": 0.5,
                      "retard_commande": 0.5, "graine": graine}
        session = Session(env, simulation=simulation, dossier_logs=dossier,
                          fichier_comm=dossier / "communication.json")
        enregistrer_missions(session)
        session.ouvrir_materiel()
        session.configurer_communication({"ip": "127.0.0.1", "port": 6800,
                                          "mon_id": ID_MOI, "autre_id": ID_FANTOME})

        stats = {"dist_min": None, "dist_min_avant_depart_fantome": None,
                 "moteurs_pendant_perte": 0, "trace": []}
        monde = session.monde

        def surveiller():
            derniere_trace = -1.0
            while not arret.is_set():
                maintenant = time.monotonic()
                if fantome.t_go is None and session.phase not in PHASES_AVANT_DEPART:
                    fantome.t_go = maintenant
                vrai = np.array(monde.position_locale())
                if reactif and fantome.t_go is not None:
                    fantome.avancer(maintenant, vrai)
                gx, gy = fantome.position(maintenant)
                d = math.hypot(vrai[0] - gx, vrai[1] - gy)
                if fantome.t_go is not None:
                    if stats["dist_min"] is None or d < stats["dist_min"]:
                        stats["dist_min"] = d
                    if nom == "occupe" and (fantome.t_part is None
                                            or maintenant < fantome.t_part):
                        if (stats["dist_min_avant_depart_fantome"] is None
                                or d < stats["dist_min_avant_depart_fantome"]):
                            stats["dist_min_avant_depart_fantome"] = d
                    if nom == "occupe" and fantome.t_part is None and \
                            math.hypot(vrai[0] - B[0], vrai[1] - B[1]) < 15.0:
                        fantome.t_part = maintenant + 30.0
                    if nom == "lien":
                        ecoule = maintenant - fantome.t_go
                        if COUPURE[0] + 4.5 <= ecoule <= COUPURE[1]:
                            if session._moteurs() not in ([0, 0], None):
                                stats["moteurs_pendant_perte"] += 1
                    if maintenant - derniere_trace >= 0.5:
                        derniere_trace = maintenant
                        stats["trace"].append((maintenant - fantome.t_go, vrai[0], vrai[1],
                                               gx, gy, d))
                time.sleep(0.1)

        fil = threading.Thread(target=surveiller, name="surveillance")
        fil.daemon = True
        fil.start()

        classe = session.missions["abba"]
        brut = {"depart_lat": points["A"]["lat"], "depart_lon": points["A"]["lon"],
                "arrivee_lat": points["B"]["lat"], "arrivee_lon": points["B"]["lon"],
                "attente": 3.0}
        brut.update(supplementaires or {})
        parametres = session.valider_parametres(classe, brut)
        debut = time.monotonic()
        code = session.executer("abba", parametres)
        duree = time.monotonic() - debut
        resultat = session.info_mission.get("resultat")
        arret.set()
        fil.join(timeout=2.0)
    finally:
        arret.set()
        if session is not None:
            session.fermer_materiel()

    with open(str(dossier / "trace.csv"), "w", encoding="utf-8") as f:
        f.write("t,x,y,fx,fy,dist\n")
        for ligne in stats["trace"]:
            f.write(",".join("{:.2f}".format(v) for v in ligne) + "\n")

    evenements = lire_journal(dossier)
    types = {}
    for e in evenements:
        types[e["type"]] = types.get(e["type"], 0) + 1
    deux_negatifs = sum(
        1 for e in evenements
        if e["type"] == "nav" and e.get("moteurs") and e["moteurs"][0] < 0
        and e["moteurs"][1] < 0)
    bilan = [e for e in evenements if e["type"] == "bilan"]
    sortie = {
        "scenario": nom, "graine": graine, "code": code, "resultat": resultat,
        "duree": round(duree, 1),
        "dist_min": stats["dist_min"],
        "dist_min_avant_depart_fantome": stats["dist_min_avant_depart_fantome"],
        "moteurs_pendant_perte": stats["moteurs_pendant_perte"],
        "deux_moteurs_negatifs": deux_negatifs,
        "evenements": types,
        "bilan": bilan[-1] if bilan else None,
        "journal": str(dossier),
    }
    return sortie


def verifier(sortie):
    """Liste des raisons d'echec (vide si le scenario passe)."""
    nom = sortie["scenario"]
    ev = sortie["evenements"]
    echecs = []
    if sortie["code"] != 0 or sortie["resultat"] != "termine":
        echecs.append("resultat {}".format(sortie["resultat"]))
    if ev.get("exception"):
        echecs.append("exception dans le journal")
    if sortie["deux_moteurs_negatifs"]:
        echecs.append("deux moteurs en arriere")
    if sortie["dist_min"] is None or sortie["dist_min"] < 2.0:
        echecs.append("collision (dist min {})".format(sortie["dist_min"]))
    if nom in ("face", "croise") and (
            sortie["dist_min"] is None or sortie["dist_min"] <= 5.0):
        echecs.append("dist min {:.1f} m <= 5 m".format(sortie["dist_min"] or 0.0))
    if nom == "occupe":
        d = sortie["dist_min_avant_depart_fantome"]
        if d is None or d < 8.0:
            echecs.append("approche a {} m du but occupe (< 8 m)".format(d))
        for cle in ("but_occupe", "but_occupe_fin", "arrivee"):
            if not ev.get(cle):
                echecs.append("evenement {} absent".format(cle))
    if nom == "lien":
        for cle in ("lien_perdu", "lien_retabli"):
            if not ev.get(cle):
                echecs.append("evenement {} absent".format(cle))
        if sortie["moteurs_pendant_perte"]:
            echecs.append("moteurs allumes pendant la perte du lien")
    return echecs


def resume(sortie, echecs):
    b = sortie.get("bilan") or {}
    verdict = "OK" if not echecs else ("INFO" if sortie["scenario"] in INFORMATIFS else "ECHEC")
    return "{:13s} graine {} : {:5s} {:>8s} {:5.0f} s  dist min {:>5s} m  urgences {}  evitement {:.0f} s".format(
        sortie["scenario"], sortie["graine"], verdict,
        str(sortie["resultat"]), sortie["duree"],
        "{:.1f}".format(sortie["dist_min"]) if sortie["dist_min"] is not None else "-",
        b.get("arrets_urgence", "-"), b.get("duree_evitement", 0.0)) + (
            "   <- " + "; ".join(echecs) if echecs else "")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--scenario", default=None,
                        help="un scenario ({}), sinon tous".format(
                            ", ".join(SCENARIOS + INFORMATIFS)))
    parser.add_argument("--informatifs", action="store_true",
                        help="ajoute les variantes passives (collision attendue)")
    parser.add_argument("--acceleration", type=float, default=20.0)
    parser.add_argument("--graines", default="1",
                        help="graines du simulateur (bruit GPS/compas), ex. 1,2,3")
    parser.add_argument("--sans-evitement", action="store_true",
                        help="desactive la repulsion (verifie la collision du scenario)")
    parser.add_argument("--param", action="append", default=[], metavar="CLE=VALEUR",
                        help="parametre de la mission (repetable)")
    parser.add_argument("--interne", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    graines = [int(g) for g in args.graines.split(",")]

    if args.interne:
        sortie = executer_scenario(args.scenario, args.acceleration, graines[0],
                                   args.sans_evitement,
                                   dict(p.split("=", 1) for p in args.param))
        print("RESULTAT " + json.dumps(sortie))
        return 0

    noms = [args.scenario] if args.scenario else SCENARIOS + (
        INFORMATIFS if args.informatifs else [])
    echecs_total = 0
    for nom in noms:
        for graine in graines:
            commande = [sys.executable, str(Path(__file__).resolve()), "--interne",
                        "--scenario", nom, "--acceleration", str(args.acceleration),
                        "--graines", str(graine)]
            if args.sans_evitement:
                commande.append("--sans-evitement")
            for p in args.param:
                commande += ["--param", p]
            processus = subprocess.run(commande, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, universal_newlines=True)
            lignes = [l for l in processus.stdout.splitlines() if l.startswith("RESULTAT ")]
            if processus.returncode != 0 or not lignes:
                print("{:13s} graine {} : ECHEC du script".format(nom, graine))
                print(processus.stdout[-2000:])
                echecs_total += 1
                continue
            sortie = json.loads(lignes[-1][len("RESULTAT "):])
            echecs = verifier(sortie)
            if echecs and nom not in INFORMATIFS:
                echecs_total += 1
            print(resume(sortie, echecs))
    print("{} scenario(s) en echec".format(echecs_total))
    return 1 if echecs_total else 0


if __name__ == "__main__":
    sys.exit(main())
