import math
import subprocess
import sys
import threading
import time
from pathlib import Path
import numpy as np

# Import compatible avec les deux usages :
#   - depuis la racine du dépôt : from drivers.ddboatclass import DDBoat
#   - depuis le dossier drivers/ : from ddboatclass import DDBoat
if __package__:
    from .arduino_driver_v2 import ArduinoIO
    from .gps_driver_v2 import GpsIO
    from .imu9_driver_v2 import Imu9IO
    from .cap_sender_v3 import compute_heading, load_calibration
    from .derive import EstimateurDerive
else:
    from arduino_driver_v2 import ArduinoIO
    from gps_driver_v2 import GpsIO
    from imu9_driver_v2 import Imu9IO
    from cap_sender_v3 import compute_heading, load_calibration
    from derive import EstimateurDerive

DRIVERS_DIR = Path(__file__).resolve().parent


class ArretDemande(Exception):
    """Levee dans les boucles de navigation quand l'arret est demande (IHM)."""


def angle_diff(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))

def vector_to_heading_cartesian(x, y):
  """Convertit un vecteur (x, y) en angle cartésien (radians).

  0 = Axe X positif, pi/2 = Axe Y positif.
  """
  return math.atan2(y, x)

def calculer_distance_gps(lat1, lon1, lat2, lon2):
    # Rayon moyen de la Terre en mètres
    R = 6371000.0

    # Conversion des degrés en radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Différences de coordonnées
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    # Formule de Haversine
    a = ( math.sin(dlat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Distance en mètres
    distance = R * c
    return distance


def dist_cart(A,B):
    yA = A[0]
    xA = A[1]
    yB = B[0]
    xB = B[1]
    return math.sqrt((yA - yB) ** 2 + (xA - xB) ** 2)


def nmea_vers_degres(valeur, hemisphere):
    """Convertit une coordonnee NMEA (ddmm.mmmm / dddmm.mmmm) en degres.

    Le GPS ne donne pas des degres decimaux : 4811.932 signifie
    48 deg 11.932 min, soit 48.19887 deg. L'hemisphere S ou W rend la
    valeur negative (la longitude du lac est donc -3.01...).
    """
    brut = float(valeur)
    degres = int(brut // 100)
    minutes = brut - degres * 100
    decimal = degres + minutes / 60.0
    if hemisphere.upper() in ("S", "W"):
        decimal = -decimal
    return decimal


def parser_trame_gll(trame):
    """Retourne (lat, lon) en degres decimaux pour une trame GLL valide.

    Retourne None si la trame est tronquee (checksum faux), si ce n'est pas
    une trame GLL ou si le GPS n'a pas de fix (statut different de 'A').
    """
    if not trame.startswith("$") or "*" not in trame:
        return None
    corps, somme = trame[1:].rsplit("*", 1)
    try:
        attendu = int(somme[:2], 16)
    except ValueError:
        return None
    calcule = 0
    for caractere in corps:
        calcule ^= ord(caractere)
    if calcule != attendu:
        return None

    champs = corps.split(",")
    if len(champs) < 7 or not champs[0].endswith("GLL"):
        return None
    if champs[6] != "A":
        return None
    try:
        lat = nmea_vers_degres(champs[1], champs[2])
        lon = nmea_vers_degres(champs[3], champs[4])
    except ValueError:
        return None
    return lat, lon


def heure_trame_gll(trame):
    """Heure UTC (secondes depuis minuit) d'une trame GLL, sinon None."""
    champs = trame.split(",")
    if len(champs) < 6 or len(champs[5]) < 6:
        return None
    try:
        return (int(champs[5][0:2]) * 3600.0 + int(champs[5][2:4]) * 60.0
                + float(champs[5][4:]))
    except ValueError:
        return None


def parser_trame_gga(trame):
    """Retourne (qualite, satellites, hdop) d'une trame GGA, sinon None."""
    if not trame.startswith("$") or "*" not in trame:
        return None
    corps, somme = trame[1:].rsplit("*", 1)
    try:
        attendu = int(somme[:2], 16)
    except ValueError:
        return None
    calcule = 0
    for caractere in corps:
        calcule ^= ord(caractere)
    if calcule != attendu:
        return None
    champs = corps.split(",")
    if len(champs) < 9:
        return None
    try:
        qualite = int(champs[6]) if champs[6] else 0
        satellites = int(champs[7]) if champs[7] else 0
        hdop = float(champs[8]) if champs[8] else None
    except ValueError:
        return None
    return qualite, satellites, hdop


class DDBoat:


    VITESSE_ROTATION = 60               # consigne moteur (0-255)
    TOLERANCE_CAP = math.radians(10.0)  # ecart accepte autour du cap cible
    TIMEOUT_PALIER = 15.0               # duree max pour atteindre un cap (s)
    TEMPS_STABILISATION = 0.00          # pause moteurs arretes entre deux caps (s)
    # Sens du tour : +1 = horaire (defaut), -1 = anti-horaire.
    SENS_TOUR = 1
    # Polarite des moteurs : +1 si (-VITESSE, +VITESSE) augmente le cap
    # mesure, -1 si cette commande le fait diminuer. Le cap du capteur
    # augmentant dans le sens horaire, la valeur par defaut est -1.
    SENS_ROTATION = -1

    # Regulateur de cap (voir _regulateur_cap). Unites : commande moteur
    # (0-255) par radian d'erreur, ou par radian.seconde, ou par rad/s.
    # Le journal de la mission du 20/10 montrait un ecart de cap permanent de
    # 12 a 25 deg avec Kp = 25 sous le vent : le terme integral le supprime.
    KP_CAP = 80.0
    KI_CAP = 12.0
    KD_CAP = 22.0                       # amortissement sur la vitesse de rotation
    LIMITE_INTEGRALE = 40.0             # correction integrale maximale (commande)
    LIMITE_DERIVEE = 30.0
    ZONE_INTEGRALE = math.radians(45.0)  # pas d'integration au-dela de cet ecart
    OUBLI_INTEGRALE = 1.0               # constante d'oubli pendant un grand virage (s)
    TAU_DERIVEE = 0.4                   # filtre de la vitesse de rotation (s)
    VITESSE_CROISIERE = 150             # commande moteur moyenne en route
    COMMANDE_MAX = 191
    # Compensation de la derive (vent, courant) par un cap de « crabe ».
    COMPENSATION_DERIVE = True

    def __init__(self, ard, gps, imu) -> None:
        self.ard = ard
        self.gps = gps
        self.imu = imu
        self.invA = None
        self.xbar = None

        # Etat du GPS : tampon des octets recus et dernier fix valide.
        # Le verrou permet de lire le GPS depuis un second thread
        # (telemetrie vers l'IHM) sans melanger les trames.
        self._tampon_gps = b""
        self._verrou_gps = threading.Lock()
        self.derniere_position_gps = None   # (lat, lon) en degres decimaux
        self.instant_fix_gps = None         # time.monotonic() du dernier fix
        self.compteur_fix_gps = 0           # nombre de fix recus

        self.derniere_trame_gll = None      # derniere trame GLL valide (texte)
        self.gga = None                     # (qualite, satellites, hdop)
        self.trames_invalides = 0           # trames GLL/GGA rejetees

        # Dernieres valeurs utiles pour la telemetrie (en radians).
        self.dernier_cap = None
        self.dernier_cap_cible = None
        self.dernier_mag_brut = None

        # Journal optionnel : fonction appelee avec (type, dict). Laisse a
        # None, aucune donnee n'est enregistree.
        self.journal = None

        # Arret demande depuis l'IHM : les boucles de navigation levent
        # ArretDemande quand cet evenement est positionne.
        self.arret = threading.Event()

        # Lecture du compas depuis plusieurs threads (mission, veille).
        self._verrou_imu = threading.Lock()
        # Cap et commande moyens entre deux fix GPS, pour estimer la derive.
        self._verrou_cap = threading.Lock()
        self._acc_cap = [0.0, 0.0, 0]       # somme sin, somme cos, nombre
        self._acc_commande = [0.0, 0]       # somme des commandes moyennes, nombre
        self._fix_precedent = None          # (lat, lon, heure_utc, instant)
        self.derive = EstimateurDerive()
        self.vitesse_sol = None             # dernier calcul : dict ou None
        self.vitesse_croisiere = self.VITESSE_CROISIERE
        self.derniere_commande = (0, 0)
        self.derniere_route = None
        self.dernier_asservissement = {}
        self._reg = {"i": 0.0, "rotation": 0.0, "cap": None, "t": None}

    def _journaliser(self, type_evenement, **donnees):
        """Transmet un evenement au journal, sans jamais bloquer le bateau."""
        if self.journal is None:
            return
        try:
            self.journal(type_evenement, donnees)
        except Exception:
            pass

    def calibration(self):
        """Lance les deux acquisitions puis charge/calcule la calibration."""
        scripts = ["create_donnees_mag.py", "create_magNorthWest.py"]

        print("=== Début du processus de calibration du DDBoat ===")

        for script in scripts:
            print(script)
            try:
                subprocess.run(
                    [sys.executable, str(DRIVERS_DIR / script)],
                    check=True,
                    cwd=DRIVERS_DIR,
                )
            except subprocess.CalledProcessError as e:
                print("Une erreur est survenue lors de l'exécution de ")
                print(script)
                return False
            print(script)
            print("exécuté avec succès.")

        # Charge invA.csv / xbar.csv, ou les calcule depuis les CSV bruts.
        self.invA, self.xbar = load_calibration()

        print("=== Calibration terminée avec succès ===")
        return True

    def _ensure_calibration(self):
        if self.invA is None or self.xbar is None:
            self.invA, self.xbar = load_calibration()

    def current_cap_rad(self):
        self._ensure_calibration()
        with self._verrou_imu:
            xmag, ymag, zmag = self.imu.read_mag_raw()
        self.dernier_mag_brut = (xmag, ymag, zmag)
        heading_rad = compute_heading([xmag, ymag, zmag], self.invA, self.xbar)
        # Cap boussole : 0 = Nord, croissant dans le sens horaire.
        self.dernier_cap = heading_rad % (2 * math.pi)
        with self._verrou_cap:
            self._acc_cap[0] += math.sin(self.dernier_cap)
            self._acc_cap[1] += math.cos(self.dernier_cap)
            self._acc_cap[2] += 1
        return self.dernier_cap

    def current_cap_deg(self):
        return np.degrees(self.current_cap_rad())

    def _commander_moteurs(self, gauche, droite):
        """Envoie une commande et la memorise pour l'estimation de derive.

        Si l'arret est demande, aucune commande non nulle ne part : les
        moteurs sont coupes et ArretDemande interrompt la boucle appelante.
        """
        if self.arret.is_set() and (gauche or droite):
            self.ard.send_arduino_cmd_motor(0, 0)
            self.derniere_commande = (0, 0)
            raise ArretDemande()
        self.ard.send_arduino_cmd_motor(gauche, droite)
        self.derniere_commande = (gauche, droite)
        with self._verrou_cap:
            self._acc_commande[0] += (gauche + droite) / 2.0
            self._acc_commande[1] += 1

    def _arreter_moteurs(self):
        """Coupe les deux moteurs."""
        self._commander_moteurs(0, 0)

    def _verifier_arret(self):
        if self.arret.is_set():
            raise ArretDemande()

    def _tourner_vers_cap(self, cible):
        """Fait tourner le bateau jusqu'au cap cible.

        Retourne True si le cap est atteint, False si le temps maximum
        est depasse.
        """
        cible %= 2 * math.pi
        self.dernier_cap_cible = cible
        debut = time.monotonic()

        while True:
            self._verifier_arret()
            current = self.current_cap_rad()

            # angle_diff() donne l'ecart le plus court entre les deux caps,
            # dans l'intervalle [-pi, pi]. C'est indispensable car les angles
            # rebouclent a 2*pi (359 degres est proche de 1 degre).
            erreur = angle_diff(cible, current)

            if abs(erreur) <= self.TOLERANCE_CAP:
                self._arreter_moteurs()
                print("Cap atteint : {:.1f} deg (ecart {:.1f} deg)".format(
                    math.degrees(current), math.degrees(erreur)))
                self._journaliser("tour_palier", atteint=True,
                                  cible=math.degrees(cible),
                                  cap=math.degrees(current),
                                  duree=time.monotonic() - debut)
                return True

            if time.monotonic() - debut > self.TIMEOUT_PALIER:
                self._arreter_moteurs()
                print("Echec : cap cible {:.1f} deg, cap actuel {:.1f} deg".format(
                    math.degrees(cible), math.degrees(current)))
                self._journaliser("tour_palier", atteint=False,
                                  cible=math.degrees(cible),
                                  cap=math.degrees(current),
                                  duree=time.monotonic() - debut)
                return False

            # Le cap du capteur augmente dans le sens horaire.
            # Une erreur positive signifie donc qu'il faut tourner a droite.
            # SENS_ROTATION adapte la commande au sens physique des moteurs.
            if erreur * self.SENS_ROTATION > 0:
                commande = (-self.VITESSE_ROTATION, self.VITESSE_ROTATION)
            else:
                commande = (self.VITESSE_ROTATION, -self.VITESSE_ROTATION)
            self._commander_moteurs(commande[0], commande[1])
            self._journaliser("tour", cible=math.degrees(cible),
                              cap=math.degrees(current),
                              erreur=math.degrees(erreur),
                              moteurs=commande, mag=self.dernier_mag_brut)

            # Petite pause pour ne pas saturer le processeur du Raspberry Pi.
            time.sleep(0.05)

    def tour_complet(self):
        """Fait faire au bateau un tour complet de 360 degres.

        Par defaut (SENS_TOUR = 1), le bateau tourne dans le sens horaire :
        cap de depart, +90 (droite), +180 (derriere), +270 (gauche), puis
        +360 degres (retour au cap initial).
        """
        cap_initial = self.current_cap_rad()
        offsets = [0.0, math.pi / 2, math.pi, 3 * math.pi / 2, 2 * math.pi]

        # Sens horaire par defaut : +90 = droite, +180 = derriere,
        # +270 = gauche, +360 = retour au cap initial.
        if self.SENS_TOUR > 0:
            cadran_dir = ["DEVANT", "DROITE", "DERRIERE", "GAUCHE", "DEVANT"]
        else:
            offsets = [-offset for offset in offsets]
            cadran_dir = ["DEVANT", "GAUCHE", "DERRIERE", "DROITE", "DEVANT"]

        print("=== Debut du tour complet ===")
        print("Cap initial : {:.1f} deg".format(math.degrees(cap_initial)))
        self._journaliser("tour_debut", cap_initial=math.degrees(cap_initial),
                          sens=self.SENS_TOUR)

        try:
            for i, offset in enumerate(offsets):
                cible = (cap_initial + offset) % (2 * math.pi)
                print("--> Cap cible {} : {:.1f} deg".format(
                    cadran_dir[i], math.degrees(cible)))

                if not self._tourner_vers_cap(cible):
                    print("Tour complet interrompu.")
                    return False

                # On laisse le bateau se stabiliser, moteurs arretes, avant
                # de mesurer le cap suivant.
                time.sleep(self.TEMPS_STABILISATION)

            print("=== Tour complet termine ===")
            return True
        finally:
            # Securite : on coupe les moteurs dans tous les cas, meme si une
            # exception survient pendant la rotation.
            self._arreter_moteurs()

    def convert_cart_position(self, lat, long, origin, angle_rotation=0.0):
        """Fait convertir la position dans le repère local du Lac"""

        lat_orig, lon_orig = origin[0], origin[1]

        # Rayon moyen de la Terre en mètres
        R = 6371000.0

        # 1. Conversion des coordonnées en radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(long)
        lat_orig_rad = math.radians(lat_orig)
        lon_orig_rad = math.radians(lon_orig)

        # 2. Calcul des distances en mètres (Repère Est / Nord brut)
        # dy = distance vers le Nord
        dy = (lat_rad - lat_orig_rad) * R
        # dx = distance vers l'Est (pondérée par le cosinus de la latitude)
        dx = (lon_rad - lon_orig_rad) * R * math.cos(lat_orig_rad)

        # 3. Application d'une rotation si votre repère local est orienté
        # (Par exemple si l'axe X de votre lac n'est pas strictement orienté vers l'Est géographique)
        if angle_rotation != 0.0:
            x_rot = dx * math.cos(angle_rotation) + dy * math.sin(angle_rotation)
            y_rot = -dx * math.sin(angle_rotation) + dy * math.cos(angle_rotation)
            return x_rot, y_rot

        return dx, dy







    def mise_a_jour_gps(self):
        """Lit toutes les trames GPS disponibles, sans bloquer.

        L'ancienne lecture (read_gll_non_blocking) lisait une seule ligne
        avec un timeout de 10 ms : la ligne pouvait etre tronquee, et la
        plupart du temps ce n'etait pas une trame GLL. Elle renvoyait alors
        [0., 'N', 0., 'W', 0.], c'est-a-dire une position a l'equateur.

        Ici les octets recus sont accumules dans un tampon et decoupes en
        lignes completes. Seules les trames GLL au checksum correct et avec
        un fix valide mettent a jour la derniere position connue.

        Retourne True si une nouvelle position a ete recue.
        """
        if self.gps is None:
            return False

        nouvelle = False
        with self._verrou_gps:
            ser = self.gps.ser
            try:
                if hasattr(ser, "in_waiting"):
                    disponibles = ser.in_waiting
                else:
                    disponibles = ser.inWaiting()
                if disponibles > 0:
                    self._tampon_gps += ser.read(disponibles)
            except Exception as exc:
                print("Erreur de lecture GPS : {}".format(exc))
                self._journaliser("gps_erreur", erreur=repr(exc))
                return False

            lignes = self._tampon_gps.split(b"\n")
            # La derniere ligne est incomplete : on la garde pour la suite.
            self._tampon_gps = lignes.pop()
            if len(self._tampon_gps) > 512:
                self._tampon_gps = b""

            for brute in lignes:
                trame = brute.decode("ascii", "ignore").strip()
                entete = trame.split(",", 1)[0]
                if entete.endswith("GGA"):
                    # Qualite du fix, pour diagnostiquer le GPS dans le journal.
                    gga = parser_trame_gga(trame)
                    if gga is None:
                        self.trames_invalides += 1
                        self._journaliser("gps_trame_rejetee", trame=trame)
                    else:
                        self.gga = gga
                    continue
                if not entete.endswith("GLL"):
                    continue
                position = parser_trame_gll(trame)
                if position is None:
                    # Trame tronquee, checksum faux ou pas de fix.
                    self.trames_invalides += 1
                    self._journaliser("gps_trame_rejetee", trame=trame)
                    continue
                self.derniere_position_gps = position
                self.derniere_trame_gll = trame
                self.instant_fix_gps = time.monotonic()
                self.compteur_fix_gps += 1
                nouvelle = True
                self._nouveau_fix(position, heure_trame_gll(trame),
                                  self.instant_fix_gps)
        return nouvelle

    def _nouveau_fix(self, position, heure_utc, instant):
        """Vitesse sol du dernier intervalle, et mesure pour l'estimateur de derive."""
        lat, lon = position
        precedent = self._fix_precedent
        self._fix_precedent = (lat, lon, heure_utc, instant)
        with self._verrou_cap:
            somme_sin, somme_cos, n_cap = self._acc_cap
            somme_cmd, n_cmd = self._acc_commande
            self._acc_cap = [0.0, 0.0, 0]
            self._acc_commande = [0.0, 0]
        if precedent is None:
            return
        if heure_utc is not None and precedent[2] is not None:
            duree = (heure_utc - precedent[2]) % 86400.0
        else:
            duree = instant - precedent[3]
        if not (0.4 <= duree <= 3.0):
            return
        rayon = 6371000.0
        nord = math.radians(lat - precedent[0]) * rayon
        est = (math.radians(lon - precedent[1]) * rayon
               * math.cos(math.radians(precedent[0])))
        v_est, v_nord = est / duree, nord / duree
        vitesse = math.hypot(v_est, v_nord)
        self.vitesse_sol = {
            "est": v_est, "nord": v_nord, "vitesse": vitesse,
            "cog": math.degrees(math.atan2(v_est, v_nord)) % 360.0,
            "instant": instant,
        }
        if n_cap < 3:
            return
        cap_moyen = math.atan2(somme_sin, somme_cos)
        longueur = min(1.0, math.hypot(somme_sin, somme_cos) / n_cap)
        dispersion = math.sqrt(max(0.0, -2.0 * math.log(max(longueur, 1e-9))))
        commande = somme_cmd / n_cmd if n_cmd else 0.0
        self.vitesse_sol["cap_moyen"] = math.degrees(cap_moyen) % 360.0
        self.vitesse_sol["commande"] = commande
        self.derive.ajouter(instant, duree, v_est, v_nord, cap_moyen,
                            dispersion, commande)

    def age_fix_gps(self):
        """Age du dernier fix GPS en secondes, ou None si aucun fix."""
        if self.instant_fix_gps is None:
            return None
        return time.monotonic() - self.instant_fix_gps

    def get_DDboat_cart_position(self, origin, angle_rotation=0.0):
        """Convertit la derniere position GPS en coordonnees locales (x, y).

        origin : tuple (lat_origin, lon_origin) en degres
        angle_rotation : angle optionnel (en radians) pour orienter le repere local

        Retourne None tant qu'aucun fix GPS valide n'a ete recu.

        Correction : les valeurs NMEA (ddmm.mmmm) etaient utilisees comme des
        degres decimaux et l'hemisphere W etait ignore. La position calculee
        etait donc a des milliers de kilometres de la base, et la direction
        donnee par Environnement.direction() n'avait aucun sens.
        """
        self.mise_a_jour_gps()
        if self.derniere_position_gps is None:
            return None
        lat, lon = self.derniere_position_gps
        return self.convert_cart_position(lat, lon, origin, angle_rotation)

    def attendre_position(self, origin, angle_rotation=0.0, timeout=120.0):
        """Attend un premier fix GPS. Retourne (x, y) ou None apres timeout."""
        debut = time.monotonic()
        dernier_message = debut
        while time.monotonic() - debut < timeout:
            position = self.get_DDboat_cart_position(origin, angle_rotation)
            if position is not None:
                return position
            if time.monotonic() - dernier_message > 5.0:
                print("En attente d'un fix GPS...")
                dernier_message = time.monotonic()
            time.sleep(0.1)
        return None

    def reinitialiser_asservissement(self):
        """Remet a zero l'integrale et la derivee du regulateur de cap."""
        self._reg = {"i": 0.0, "rotation": 0.0, "cap": None, "t": None}

    def _regulateur_cap(self, erreur, cap):
        """Regulateur PID de l'ecart de cap (rad). Retourne (u, detail).

        u > 0 : il faut augmenter le cap (tourner a droite).
        - P : proportionnel a l'ecart ;
        - I : supprime l'ecart permanent cause par le vent ou par des
          moteurs de poussee differente. Il est borne (anti-emballement) et
          gele quand l'ecart est grand ou la commande saturee ;
        - D : amortit avec la vitesse de rotation mesuree (filtree), plutot
          qu'avec la derivee de la consigne, pour ne pas reagir aux sauts de
          route.
        """
        reg = self._reg
        maintenant = time.monotonic()
        dt = None
        if reg["t"] is not None:
            dt = maintenant - reg["t"]
            if dt <= 0.0 or dt > 1.0:
                # Boucle interrompue : on repart sans memoire.
                reg["i"] = 0.0
                reg["rotation"] = 0.0
                dt = None
        if dt is not None and reg["cap"] is not None:
            vitesse = angle_diff(cap, reg["cap"]) / dt
            alpha = dt / (self.TAU_DERIVEE + dt)
            reg["rotation"] += (vitesse - reg["rotation"]) * alpha
        reg["t"] = maintenant
        reg["cap"] = cap

        u_p = self.KP_CAP * erreur
        if dt is not None and abs(erreur) >= self.ZONE_INTEGRALE:
            # Grand virage : la correction apprise pour l'ancien cap (couple
            # du vent) ne vaut plus pour le nouveau, on l'oublie.
            reg["i"] *= math.exp(-dt / self.OUBLI_INTEGRALE)
        if dt is not None and abs(erreur) < self.ZONE_INTEGRALE:
            reg["i"] += erreur * dt
            if self.KI_CAP > 0.0:
                limite = self.LIMITE_INTEGRALE / self.KI_CAP
                reg["i"] = max(-limite, min(limite, reg["i"]))
        u_i = self.KI_CAP * reg["i"]
        u_d = -self.KD_CAP * reg["rotation"]
        u_d = max(-self.LIMITE_DERIVEE, min(self.LIMITE_DERIVEE, u_d))
        u = u_p + u_i + u_d
        return u, {"u_p": u_p, "u_i": u_i, "u_d": u_d,
                   "rotation": math.degrees(reg["rotation"])}

    def _melanger(self, u, vitesse):
        """Commandes gauche/droite pour une correction u autour de `vitesse`.

        Quand un moteur sature, on baisse l'autre du meme montant : la
        difference de poussee (donc la rotation) est conservee, seule la
        vitesse diminue. L'ancien ecretage simple annulait la correction.
        """
        u = max(-self.COMMANDE_MAX / 2.0, min(self.COMMANDE_MAX / 2.0, u))
        gauche = vitesse - self.SENS_ROTATION * u
        droite = vitesse + self.SENS_ROTATION * u
        exces = max(gauche, droite) - self.COMMANDE_MAX
        if exces > 0:
            gauche -= exces
            droite -= exces
        manque = -min(gauche, droite)
        if manque > 0:
            gauche += manque
            droite += manque
        gauche = max(0.0, min(self.COMMANDE_MAX, gauche))
        droite = max(0.0, min(self.COMMANDE_MAX, droite))
        return gauche, droite

    def suivre_route(self, route, derive=None, vitesse=None):
        """Applique les moteurs pour que la route sol soit `route` (cap boussole, rad).

        Avec la compensation de derive, le bateau pointe a cote de la route
        (cap de crabe) pour que le vent ne l'en ecarte pas. Retourne
        (cap_commande, cap_actuel, moteur_gauche, moteur_droit).
        """
        if derive is None:
            derive = self.COMPENSATION_DERIVE
        if vitesse is None:
            vitesse = self.vitesse_croisiere
        route %= 2 * math.pi
        current_cap = self.current_cap_rad()

        crabe = 0.0
        estimation = None
        cap_commande = route
        if derive:
            cap_commande, crabe, estimation = self.derive.cap_pour_route(
                route, time.monotonic())
        self.derniere_route = route
        self.dernier_cap_cible = cap_commande

        erreur = angle_diff(cap_commande, current_cap)
        u, detail = self._regulateur_cap(erreur, current_cap)
        gauche, droite = self._melanger(u, vitesse)
        self._commander_moteurs(gauche, droite)

        detail.update({
            "route": math.degrees(route),
            "crabe": math.degrees(crabe),
            "derive_active": bool(derive),
        })
        if estimation is not None:
            detail.update({"derive_est": estimation["est"],
                           "derive_nord": estimation["nord"],
                           "s_eau": estimation["s"],
                           "derive_n": estimation["n"],
                           "derive_rms": estimation["rms"]})
        self.dernier_asservissement = detail
        return cap_commande, current_cap, gauche, droite

    def maintien_cap(self, target_dir, angle_rotation=0.0, derive=None,
                     vitesse=None):
        """Calcule et applique la vitesse des moteurs pour suivre target_dir.

        target_dir est un vecteur (x, y) du repere local du lac.
        Retourne (cap_cible, cap_actuel, moteur_gauche, moteur_droit) ;
        cap_cible est le cap commande (route + crabe si derive compensee).
        """
        # Convention d'angle : current_cap_rad() est un cap boussole (0 =
        # Nord, sens horaire), alors que atan2(y, x) donne un angle
        # mathematique (0 = Est, sens anti-horaire). Le cap boussole d'un
        # vecteur (Est, Nord) est atan2(Est, Nord).
        vx = float(target_dir[0])
        vy = float(target_dir[1])
        if angle_rotation != 0.0:
            # Repere local tourne : on ramene le vecteur en (Est, Nord).
            est = vx * math.cos(angle_rotation) - vy * math.sin(angle_rotation)
            nord = vx * math.sin(angle_rotation) + vy * math.cos(angle_rotation)
        else:
            est, nord = vx, vy
        route = math.atan2(est, nord) % (2 * math.pi)
        return self.suivre_route(route, derive=derive, vitesse=vitesse)

    def homing(self, Environnement, rayon_arrivee=5.0, timeout=None,
               age_fix_max=3.0):
        """Ramene le bateau a la base en suivant Environnement.direction().

        Retourne True si la base est atteinte, False si timeout est depasse.
        """
        debut = time.monotonic()
        gps_perime = False
        self.reinitialiser_asservissement()
        try:
            while True:
                self._verifier_arret()
                if timeout is not None and time.monotonic() - debut > timeout:
                    print("Homing : temps maximal depasse.")
                    self._journaliser("homing_fin", arrive=False,
                                      raison="timeout")
                    return False

                # Une seule lecture par iteration, dans le meme repere (avec
                # rotation) que la base, le couloir et les bordures. Avant,
                # la distance utilisait la rotation mais pas la direction.
                position = self.get_DDboat_cart_position(
                    Environnement.home_latlong, Environnement.rotation)
                age = self.age_fix_gps()
                if position is None or age is None or age > age_fix_max:
                    # Pas de position fiable : on ne navigue pas a l'aveugle.
                    self._arreter_moteurs()
                    if not gps_perime:
                        self._journaliser("gps_perime", age=age)
                        gps_perime = True
                    time.sleep(0.2)
                    continue
                if gps_perime:
                    self._journaliser("gps_retabli", age=age)
                    gps_perime = False

                x, y = position
                distance = dist_cart((x, y), Environnement.home)
                if distance <= rayon_arrivee:
                    print("Base atteinte.")
                    self._journaliser("homing_fin", arrive=True, x=x, y=y,
                                      distance=distance)
                    return True

                direction = Environnement.direction(x, y)
                cible, cap, gauche, droite = self.maintien_cap(
                    direction, Environnement.rotation)
                self._journaliser("homing", x=x, y=y, age=age,
                                  distance=distance,
                                  direction=(float(direction[0]),
                                             float(direction[1])),
                                  cap_cible=math.degrees(cible),
                                  cap=math.degrees(cap),
                                  erreur=math.degrees(angle_diff(cible, cap)),
                                  moteurs=(gauche, droite),
                                  mag=self.dernier_mag_brut,
                                  **self.dernier_asservissement)
                time.sleep(0.05)
        finally:
            # Securite : moteurs coupes a l'arrivee comme en cas d'erreur.
            self._arreter_moteurs()


    def rejoindre_keeping_zone(self, pos_keep_point, origin):
        tolerance = 2
        while True:
            pos = self.get_DDboat_cart_position(origin)
            if dist_cart(pos,pos_keep_point) < tolerance:
                break
            else:
                direction = pos_keep_point - pos
                ddb.maintien_cap(direction, origin)
            time.sleep(0.05)





if __name__ == "__main__":
    from environnementclass import Environnement
    from imu9_driver_v2 import Imu9IO
    from gps_driver_v2 import GpsIO
    from arduino_driver_v2 import ArduinoIO
    ard = ArduinoIO()
    gps = GpsIO()
    imu = Imu9IO()
    ddb = DDBoat(ard, gps, imu)
    
    lac = Environnement()
    position = ddb.attendre_position(lac.home_latlong, lac.rotation)
    if position is None:
        print("Aucun fix GPS : arret.")
    else:
        x, y = position
        if not lac.lim(x, y):
            print("Hors limites : tour complet puis retour a la base.")
            ddb.tour_complet()
        ddb.homing(lac)
    print("Fini!")