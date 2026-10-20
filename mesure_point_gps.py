#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mesure précise d'un point GPS depuis le bateau.

Placez le bateau sur le point à relever, puis lancez le script :

    python3 mesure_point_gps.py home
    python3 mesure_point_gps.py couloir --samples 90 --interval 0.5

Le script lit les trames NMEA GLL, ignore les positions invalides et les
mesures aberrantes, puis affiche la latitude et la longitude moyennes en
degrés décimaux, prêtes à copier dans le code.

Le point mesuré est celui de l'antenne GPS du bateau. Pour une mesure
précise, laissez le bateau immobile pendant toute l'acquisition.
"""

import argparse
import math
import os
import re
import statistics
import sys
import time
import unicodedata

R_TERRE = 6371000.0


def nmea_vers_decimal(valeur, hemisphere):
    """Convertit une latitude/longitude NMEA (ddmm.mmmm) en degrés décimaux."""
    if not valeur:
        return None

    try:
        brut = float(valeur)
    except ValueError:
        return None

    hemisphere = hemisphere.upper()
    if hemisphere not in ("N", "S", "E", "W"):
        return None

    degres = int(brut // 100)
    minutes = brut - degres * 100
    decimal = degres + minutes / 60.0
    if hemisphere in ("S", "W"):
        decimal = -decimal
    return decimal


def checksum_valide(trame):
    """Vérifie le checksum NMEA d'une trame (format $...*XX)."""
    if "*" not in trame:
        return False

    corps, somme = trame.rsplit("*", 1)
    if not corps.startswith("$"):
        return False

    try:
        attendu = int(somme[:2], 16)
    except ValueError:
        return False

    calcule = 0
    for caractere in corps[1:]:
        calcule ^= ord(caractere)
    return calcule == attendu


def parser_gll(trame):
    """Retourne (latitude, longitude) d'une trame GLL valide, sinon None."""
    champs = trame.split(",")
    if len(champs) < 7 or not champs[0].endswith("GLL"):
        return None
    # Le checksum peut être collé au champ de statut quand le mode NMEA
    # n'est pas présent : "$GPGLL,...,A*31".
    if champs[6].split("*", 1)[0] != "A":
        return None

    latitude = nmea_vers_decimal(champs[1], champs[2])
    longitude = nmea_vers_decimal(champs[3], champs[4])
    if latitude is None or longitude is None:
        return None
    return latitude, longitude


def parser_gga(trame):
    """Retourne (qualité du fix, HDOP) d'une trame GGA, sinon None."""
    champs = trame.split(",")
    if len(champs) < 9 or not champs[0].endswith("GGA"):
        return None

    try:
        qualite = int(champs[6]) if champs[6] else 0
    except ValueError:
        qualite = 0

    try:
        hdop = float(champs[8]) if champs[8] else None
    except ValueError:
        hdop = None

    return qualite, hdop


def lire_trame(gps, timeout=0.5):
    """Lit une trame NMEA, ou retourne None si rien n'est disponible."""
    try:
        recu, trame = gps.read_non_blocking(timeout=timeout)
    except Exception:
        return None

    if not recu:
        return None
    return trame.strip()


def vers_local(latitude, longitude, lat_origine, lon_origine):
    """Convertit une position en mètres (Est, Nord) autour d'une origine."""
    nord = math.radians(latitude - lat_origine) * R_TERRE
    est = (math.radians(longitude - lon_origine) * R_TERRE
           * math.cos(math.radians(lat_origine)))
    return est, nord


def collecter_mesures(gps, nombre, intervalle, timeout, hdop_max):
    """Collecte des positions GPS valides jusqu'à en avoir `nombre`."""
    echantillons = []
    rejets_fix = 0
    rejets_hdop = 0
    gga_vu = False
    derniere_qualite = None
    dernier_hdop = None
    debut = time.monotonic()
    derniere_mesure = 0.0
    dernier_message = debut

    try:
        while len(echantillons) < nombre:
            maintenant = time.monotonic()
            if timeout > 0 and maintenant - debut > timeout:
                print("\nDélai maximal de {:.1f} s atteint.".format(timeout))
                break

            trame = lire_trame(gps)
            if trame is None:
                if maintenant - dernier_message >= 5.0:
                    print("En attente d'une position GPS valide...")
                    dernier_message = maintenant
                continue

            if not checksum_valide(trame):
                continue

            champs = trame.split(",")
            if champs[0].endswith("GGA"):
                gga = parser_gga(trame)
                if gga is not None:
                    gga_vu = True
                    derniere_qualite, dernier_hdop = gga
                continue

            if not champs[0].endswith("GLL"):
                continue

            position = parser_gll(trame)
            if position is None:
                rejets_fix += 1
                continue

            if derniere_qualite is not None and derniere_qualite <= 0:
                rejets_fix += 1
                continue

            if (hdop_max > 0 and dernier_hdop is not None
                    and dernier_hdop > hdop_max):
                rejets_hdop += 1
                continue

            maintenant = time.monotonic()
            if maintenant - derniere_mesure < intervalle:
                continue

            echantillons.append((position[0], position[1], dernier_hdop))
            derniere_mesure = maintenant

            affichage = "[{:3d}/{}] lat={:.7f} lon={:.7f}".format(
                len(echantillons), nombre, position[0], position[1])
            if dernier_hdop is not None:
                affichage += "  HDOP={:.1f}".format(dernier_hdop)
            print(affichage)

    except KeyboardInterrupt:
        print("\nAcquisition interrompue par l'utilisateur.")

    return echantillons, {
        "rejets_fix": rejets_fix,
        "rejets_hdop": rejets_hdop,
        "gga_vu": gga_vu,
        "duree": time.monotonic() - debut,
    }


def calculer_resultat(echantillons, ecart_max):
    """Calcule la position moyenne en rejetant les mesures aberrantes."""
    lats = [e[0] for e in echantillons]
    lons = [e[1] for e in echantillons]
    lat_median = statistics.median(lats)
    lon_median = statistics.median(lons)

    positions = [vers_local(e[0], e[1], lat_median, lon_median)
                 for e in echantillons]
    distances = [math.hypot(x, y) for x, y in positions]
    distance_median = statistics.median(distances)
    ecarts = [abs(d - distance_median) for d in distances]
    mad = statistics.median(ecarts)

    # Seuil robuste : 3 fois l'écart absolu médian, avec un minimum de 2 m
    # pour ne pas rejeter une dispersion normale du GPS.
    seuil = max(3.0 * 1.4826 * mad, 2.0)
    if ecart_max > 0:
        seuil = min(seuil, ecart_max)

    conserves = [e for e, d in zip(echantillons, distances) if d <= seuil]
    if not conserves:
        conserves = list(echantillons)

    nombre = len(conserves)
    lat_moyenne = sum(e[0] for e in conserves) / nombre
    lon_moyenne = sum(e[1] for e in conserves) / nombre

    locaux = [vers_local(e[0], e[1], lat_moyenne, lon_moyenne)
              for e in conserves]
    x_moyen = sum(x for x, _ in locaux) / nombre
    y_moyen = sum(y for _, y in locaux) / nombre
    var_x = sum((x - x_moyen) ** 2 for x, _ in locaux) / nombre
    var_y = sum((y - y_moyen) ** 2 for _, y in locaux) / nombre
    dispersion = math.sqrt(var_x + var_y)
    erreur_type = dispersion / math.sqrt(nombre) if nombre > 1 else 0.0
    ecart_max_observe = max(math.hypot(x - x_moyen, y - y_moyen)
                            for x, y in locaux)

    hdops = [e[2] for e in conserves if e[2] is not None]
    hdop_moyen = sum(hdops) / len(hdops) if hdops else None

    return {
        "brut": len(echantillons),
        "conserves": nombre,
        "rejetes": len(echantillons) - nombre,
        "seuil": seuil,
        "lat": lat_moyenne,
        "lon": lon_moyenne,
        "dispersion": dispersion,
        "erreur_type": erreur_type,
        "ecart_max": ecart_max_observe,
        "hdop": hdop_moyen,
    }


def nom_variable(nom):
    """Transforme un nom de point en nom de variable Python."""
    sans_accents = unicodedata.normalize("NFKD", nom)
    sans_accents = sans_accents.encode("ascii", "ignore").decode("ascii")
    sans_accents = re.sub(r"[^0-9a-zA-Z]+", "_", sans_accents)
    sans_accents = sans_accents.strip("_").lower()
    return sans_accents or "point"


def afficher_resultat(nom, resultat, infos):
    """Affiche le résultat et une ligne prête à copier dans le code."""
    print("")
    print("=== Résultat ===")
    print("Point mesuré              : {}".format(nom))
    print("Échantillons valides      : {}".format(resultat["brut"]))
    print("Échantillons conservés    : {}".format(resultat["conserves"]))
    if resultat["rejetes"]:
        print("Échantillons rejetés      : {} (seuil {:.1f} m)".format(
            resultat["rejetes"], resultat["seuil"]))
    if infos["rejets_fix"] or infos["rejets_hdop"]:
        print("Trames GLL ignorées       : {} fix invalide, {} HDOP trop élevé"
              .format(infos["rejets_fix"], infos["rejets_hdop"]))
    print("Dispersion (écart-type)   : ±{:.2f} m".format(
        resultat["dispersion"]))
    print("Erreur type de la moyenne : ±{:.2f} m".format(
        resultat["erreur_type"]))
    print("Écart maximal à la moyenne: {:.2f} m".format(resultat["ecart_max"]))
    print("Durée d'acquisition       : {:.1f} s".format(infos["duree"]))

    if resultat["hdop"] is not None:
        print("HDOP moyen                : {:.2f}".format(resultat["hdop"]))
    elif infos["gga_vu"]:
        print("HDOP moyen                : indisponible")
    else:
        print("HDOP                      : non fourni par le GPS")

    latitude = resultat["lat"]
    longitude = resultat["lon"]
    variable = nom_variable(nom)

    print("")
    print("Latitude  : {:.7f}°".format(latitude))
    print("Longitude : {:.7f}°".format(longitude))
    print("")
    print("À copier dans environnementclass.py :")
    print("coord_{} = np.array([{:.7f}, {:.7f}])".format(
        variable, latitude, longitude))
    print("")
    print("Format brut : {:.7f}, {:.7f}".format(latitude, longitude))

    if resultat["dispersion"] > 3.0:
        print("")
        print("Attention : dispersion élevée, le bateau a peut-être bougé "
              "pendant la mesure.")


def ouvrir_gps(numero):
    """Ouvre le GPS quel que soit le dossier depuis lequel le script est lancé."""
    dossier_drivers = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "drivers")
    if dossier_drivers not in sys.path:
        sys.path.insert(0, dossier_drivers)

    from gps_driver_v2 import GpsIO
    return GpsIO(tty_dev=numero)


def parser_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="Mesure une position GPS précise en moyennant plusieurs "
                    "trames NMEA.")
    parser.add_argument("nom", nargs="?", default="point",
                        help="nom du point à mesurer (ex. home, couloir)")
    parser.add_argument("-n", "--samples", type=int, default=60,
                        help="nombre de positions à moyenner (défaut : 60)")
    parser.add_argument("-i", "--interval", type=float, default=0.5,
                        help="intervalle minimal entre deux mesures en "
                             "secondes (défaut : 0.5)")
    parser.add_argument("-t", "--timeout", type=float, default=180.0,
                        help="durée maximale d'acquisition en secondes, "
                             "0 pour illimité (défaut : 180)")
    parser.add_argument("--max-hdop", type=float, default=2.0,
                        help="HDOP maximal accepté, 0 pour désactiver "
                             "(défaut : 2.0)")
    parser.add_argument("--max-scatter", type=float, default=10.0,
                        help="écart maximal à la médiane en mètres, "
                             "0 pour désactiver (défaut : 10.0)")
    parser.add_argument("--gps", type=int, choices=(0, 1, 2), default=0,
                        help="périphérique GPS : 0, 1 ou 2 (défaut : 0)")

    args = parser.parse_args(argv)
    if args.samples < 1:
        parser.error("--samples doit être supérieur ou égal à 1")
    if args.interval < 0:
        parser.error("--interval doit être positif ou nul")
    if args.timeout < 0:
        parser.error("--timeout doit être positif ou nul")
    if args.max_hdop < 0:
        parser.error("--max-hdop doit être positif ou nul")
    if args.max_scatter < 0:
        parser.error("--max-scatter doit être positif ou nul")
    return args


def main(argv=None):
    args = parser_arguments(argv)

    print("=== Mesure GPS : {} ===".format(args.nom))
    print("Placez le bateau sur le point et ne le déplacez plus.")
    if args.max_hdop > 0:
        print("Acquisition de {} positions (HDOP max {:.1f})...".format(
            args.samples, args.max_hdop))
    else:
        print("Acquisition de {} positions...".format(args.samples))

    try:
        gps = ouvrir_gps(args.gps)
    except Exception as exc:
        print("Erreur : impossible d'ouvrir le GPS : {}".format(exc))
        return 1

    try:
        echantillons, infos = collecter_mesures(
            gps, args.samples, args.interval, args.timeout, args.max_hdop)
    finally:
        try:
            gps.close()
        except Exception:
            pass

    if not echantillons:
        print("Aucune position GPS valide reçue.")
        return 1

    if len(echantillons) < args.samples:
        print("Attention : seulement {} mesures valides sur {} demandées."
              .format(len(echantillons), args.samples))

    resultat = calculer_resultat(echantillons, args.max_scatter)
    afficher_resultat(args.nom, resultat, infos)
    return 0


if __name__ == "__main__":
    sys.exit(main())
