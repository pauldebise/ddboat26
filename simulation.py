# -*- coding: utf-8 -*-
"""Bateau simule : permet de lancer l'IHM et les missions sans materiel.

    python3 ihm_lac.py --bateau --simulation

Le simulateur remplace l'Arduino, le GPS et le compas par des objets qui ont
la meme interface que les vrais pilotes. Tout le reste du programme (missions,
regulateur de cap, estimation de derive, journal) tourne sans modification.

Modele physique (regle sur le journal de la mission du 20/10) :
- vitesse dans l'eau proportionnelle a la commande moyenne (1,3 m/s a 150),
  avec une inertie d'une seconde ;
- rotation proportionnelle a la difference de poussee (30 deg/s pour une
  difference de 120 en rotation sur place) ;
- moteurs de poussee legerement differente ;
- derive constante (vent, courant) et couple de vent sur le cap ;
- compas avec biais et bruit (1,2 deg) ;
- GPS a 1 Hz, bruit correle de quelques dizaines de centimetres, trames NMEA
  GLL et GGA avec checksum.

Le bateau reste amarre (immobile) jusqu'a la premiere commande moteur, comme
quand on le tient a la main pendant le compte a rebours.

L'horloge peut etre acceleree (`acceleration`) : time.monotonic() et
time.sleep() sont alors remplaces dans tout le programme.
"""

import math
import random
import signal
import sys
import threading
import time

R_TERRE = 6371000.0


_HORLOGE_REELLE = (time.monotonic, time.sleep)


def installer_horloge(acceleration):
    """Remplace time.monotonic/time.sleep par une horloge acceleree.

    Peut etre rappelee : l'acceleration s'applique toujours a l'horloge reelle.
    """
    reel_monotonic, reel_sleep = _HORLOGE_REELLE
    debut = reel_monotonic()
    acceleration = float(acceleration)

    def monotonic():
        return debut + (reel_monotonic() - debut) * acceleration

    def sleep(secondes):
        reel_sleep(max(0.0, secondes) / acceleration)

    time.monotonic = monotonic
    time.sleep = sleep


class MondeSimule(object):
    GAIN_VITESSE = 1.3 / 150.0          # m/s par unite de commande
    TAU_VITESSE = 1.0

    def __init__(self, origine, rotation=0.0, depart=(15.0, 8.0, 330.0),
                 vent=(0.30, 0.36), asymetrie=0.04, biais_compas=8.0,
                 bruit_compas=1.2, bruit_gps=0.15, couple=4.0,
                 direction_couple=180.0, sens_physique=1, graine=1,
                 retard_commande=0.25, periode_compas=0.097, gain_rotation=0.5,
                 tau_rotation=0.5, gel_vitesse=0.0):
        self.origine = (float(origine[0]), float(origine[1]))
        self.rotation = float(rotation)
        self.verrou = threading.RLock()
        self.hasard = random.Random(graine)
        x, y, cap = depart
        c, s = math.cos(self.rotation), math.sin(self.rotation)
        self.est = x * c - y * s
        self.nord = x * s + y * c
        self.cap = math.radians(cap)
        self.vitesse = 0.0
        self.rotation_cap = 0.0
        self.vent = (float(vent[0]), float(vent[1]))
        self.asymetrie = asymetrie
        self.biais_compas = math.radians(biais_compas)
        self.bruit_compas = math.radians(bruit_compas)
        self.bruit_gps = bruit_gps
        self.couple = math.radians(couple)
        self.direction_couple = math.radians(direction_couple)
        self.sens_physique = sens_physique
        self.commande = (0.0, 0.0)
        self.historique = []              # (instant, gauche, droite)
        self.retard_commande = retard_commande
        self.periode_compas = periode_compas
        self.gain_rotation = math.radians(gain_rotation)
        self.tau_rotation = tau_rotation
        self.compas = None                # (instant de l'echantillon, valeur)
        self.amarre = True
        self.erreur_gps = [0.0, 0.0]
        # Comme le GPS du bateau : sous cette vitesse sol (m/s), la position
        # reste figee jusqu'a ce que le bateau reparte (0 = pas de gel).
        self.gel_vitesse = float(gel_vitesse)
        self.derniere_mesure = None
        self.t = time.monotonic()
        self.trace = []

    def avancer(self, instant=None):
        with self.verrou:
            if instant is None:
                instant = time.monotonic()
            while self.t < instant:
                dt = min(0.02, instant - self.t)
                self._pas(dt)
                self.t += dt

    def _pas(self, dt):
        gauche, droite = self.commande_retardee()
        poussee_g = gauche * (1.0 - self.asymetrie)
        poussee_d = droite * (1.0 + self.asymetrie)
        cible_vitesse = self.GAIN_VITESSE * (poussee_g + poussee_d) / 2.0
        self.vitesse += (cible_vitesse - self.vitesse) * dt / self.TAU_VITESSE
        cible_rotation = self.sens_physique * self.gain_rotation * (poussee_g - poussee_d)
        if not self.amarre:
            cible_rotation += self.couple * math.sin(self.cap - self.direction_couple)
        self.rotation_cap += (cible_rotation - self.rotation_cap) * dt / self.tau_rotation
        self.cap += self.rotation_cap * dt
        if not self.amarre:
            self.est += (self.vitesse * math.sin(self.cap) + self.vent[0]) * dt
            self.nord += (self.vitesse * math.cos(self.cap) + self.vent[1]) * dt

    def commande_retardee(self):
        """Commande effectivement appliquee : celle d'il y a `retard_commande` s."""
        instant = self.t - self.retard_commande
        courant = (0.0, 0.0)
        for (t, g, d) in self.historique:
            if t <= instant:
                courant = (g, d)
            else:
                break
        return courant

    def commander(self, gauche, droite):
        with self.verrou:
            self.avancer()
            self.commande = (float(gauche), float(droite))
            self.historique.append((self.t, float(gauche), float(droite)))
            while len(self.historique) > 2 and self.historique[1][0] < self.t - 2.0:
                self.historique.pop(0)
            if self.amarre and abs(gauche) + abs(droite) > 60.0:
                self.amarre = False

    def cap_mesure(self):
        # Le capteur ne produit qu'un echantillon toutes les ~0,1 s (10 Hz) :
        # les lectures plus rapides repetent la meme valeur.
        with self.verrou:
            self.avancer()
            if self.compas is None or self.t - self.compas[0] >= self.periode_compas:
                valeur = (self.cap + self.biais_compas
                          + self.hasard.gauss(0.0, self.bruit_compas)) % (2 * math.pi)
                self.compas = (self.t, valeur)
            return self.compas[1]

    def position_locale(self):
        """(x, y) vrais dans le repere local."""
        with self.verrou:
            self.avancer()
            c, s = math.cos(self.rotation), math.sin(self.rotation)
            return (self.est * c + self.nord * s, -self.est * s + self.nord * c)

    def latlon_mesure(self):
        """Position GPS bruitee (erreur correlee dans le temps)."""
        with self.verrou:
            for i in (0, 1):
                self.erreur_gps[i] = (0.9 * self.erreur_gps[i]
                                      + self.hasard.gauss(0.0, self.bruit_gps))
            if self.amarre:
                ve = vn = 0.0
            else:
                ve = self.vitesse * math.sin(self.cap) + self.vent[0]
                vn = self.vitesse * math.cos(self.cap) + self.vent[1]
            if (self.gel_vitesse > 0.0 and self.derniere_mesure is not None
                    and math.hypot(ve, vn) < self.gel_vitesse):
                est, nord = self.derniere_mesure
            else:
                est = self.est + self.erreur_gps[0]
                nord = self.nord + self.erreur_gps[1]
                self.derniere_mesure = (est, nord)
        lat0 = self.origine[0]
        lat = lat0 + math.degrees(nord / R_TERRE)
        lon = self.origine[1] + math.degrees(est / (R_TERRE * math.cos(math.radians(lat0))))
        return lat, lon


def _checksum(corps):
    somme = 0
    for caractere in corps:
        somme ^= ord(caractere)
    return "{:02X}".format(somme)


def _ddmm(valeur, largeur):
    valeur = abs(valeur)
    degres = int(valeur)
    minutes = (valeur - degres) * 60.0
    return "{:0{l}d}{:07.4f}".format(degres, minutes, l=largeur)


class SerieSimulee(object):
    """Flux NMEA GLL + GGA a 1 Hz, comme le GPS du bateau."""

    def __init__(self, monde):
        self.monde = monde
        self.tampon = b""
        self.prochaine = math.ceil(time.monotonic())

    def _alimenter(self):
        maintenant = time.monotonic()
        while self.prochaine <= maintenant:
            self.monde.avancer(self.prochaine)
            lat, lon = self.monde.latlon_mesure()
            reste = int(15 * 3600 + 41 * 60 + 5 + self.prochaine)
            heure = "{:02d}{:02d}{:02d}.000".format(
                (reste // 3600) % 24, (reste // 60) % 60, reste % 60)
            champs_pos = "{},{},{},{}".format(
                _ddmm(lat, 2), "N" if lat >= 0 else "S",
                _ddmm(lon, 3), "E" if lon >= 0 else "W")
            gll = "GPGLL,{},{},A,D".format(champs_pos, heure)
            gga = "GPGGA,{},{},2,08,1.0,100.0,M,50.0,M,,".format(heure, champs_pos)
            for corps in (gga, gll):
                ligne = "${}*{}\r\n".format(corps, _checksum(corps))
                self.tampon += ligne.encode("ascii")
            self.prochaine += 1

    @property
    def in_waiting(self):
        self._alimenter()
        return len(self.tampon)

    def read(self, n):
        self._alimenter()
        sortie, self.tampon = self.tampon[:n], self.tampon[n:]
        return sortie

    def close(self):
        pass


class GpsSimule(object):
    def __init__(self, monde):
        self.ser = SerieSimulee(monde)

    def close(self):
        pass


class ImuSimule(object):
    """read_mag_raw() compatible avec invA = identite, xbar = 0."""

    def __init__(self, monde):
        self.monde = monde

    def read_mag_raw(self):
        cap = self.monde.cap_mesure()
        return (1000.0 * math.cos(cap), 1000.0 * math.sin(cap), 0.0)


class ArduinoSimule(object):
    def __init__(self, monde):
        self.monde = monde
        self.cmdl = 0
        self.cmdr = 0
        self.dirl = " "
        self.dirr = " "
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        print("You pressed Ctrl+C! set motors to 0!")
        self.send_arduino_cmd_motor(0, 0)
        sys.exit(0)

    @staticmethod
    def _borner(valeur):
        return max(-255, min(255, valeur))

    def send_arduino_cmd_motor(self, gauche, droite):
        gauche = self._borner(gauche)
        droite = self._borner(droite)
        self.cmdl, self.dirl = (-gauche, "-") if gauche < 0 else (gauche, " ")
        self.cmdr, self.dirr = (-droite, "-") if droite < 0 else (droite, " ")
        self.monde.commander(gauche, droite)


def creer_materiel(origine, rotation=0.0, acceleration=1.0, **parametres):
    """Retourne (arduino, gps, imu, monde) simules."""
    installer_horloge(acceleration)
    monde = MondeSimule(origine, rotation, **parametres)
    return ArduinoSimule(monde), GpsSimule(monde), ImuSimule(monde), monde
