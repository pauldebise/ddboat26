# -*- coding: utf-8 -*-
"""Mission capture : wrapper IHM.

La mission complete est un aller-retour du milieu de la corde :

1. les deux bateaux partent de leur position, corde entre eux ;
2. le milieu de la corde va capturer le point choisi (une bouee) ;
3. apres la capture, le milieu revient a sa position de depart, a un point
   connu de points.json, ou a un point choisi a la souris dans l'IHM.

Le wrapper ne contient pas la loi de guidage : il lit les positions de la
Session, appelle les fonctions de mission_capture.py, envoie les commandes
et journalise. La loi (normale, erreur, u_envitesse, repulsion, capture)
reste dans mission_capture.py.

Communication : le thread de Session echange les positions et les caps a
1 Hz. Le cap recu est en degres ; la loi de guidage travaille en radians.
Ici, on ne se sert pas du cap recu pour le guidage : les positions
suffisent. Le cap est seulement journalise.

Les deux bateaux doivent lancer la meme mission avec le meme point de
capture et des ids differents (14 et 18). La position de depart du milieu
est relevee au lancement ; le point de retour est soit ce milieu, soit un
point choisi dans points.json ou sur la carte.
"""

import math
import time

import numpy as np

import mission_capture as mc
from session import Mission
from ddboatclass import ArretDemande, angle_diff

CHOIX_OUI_NON = ["oui", "non"]


def _param(nom, label, defaut, **autres):
    definition = {"nom": nom, "label": label, "defaut": defaut}
    definition.update(autres)
    return definition


class MissionCapture(Mission):
    NOM = "capture"
    TITRE = "Mission capture (aller-retour de la corde)"
    DESCRIPTION = (
        "Aller-retour du milieu de la corde : capture le point choisi (une "
        "bouee), puis revient au point de depart du milieu, a un point connu "
        "de points.json ou a un point choisi sur la carte. Les deux bateaux "
        "lancent la mission avec le meme point de capture et des ids "
        "differents. La formation tourne en faisant avancer un bateau plus "
        "que l'autre ; les bateaux ne reculent pas (rayon de courbure minimal "
        "L/2). Repulsion de l'autre bateau, arret d'urgence, arret si sa "
        "position est trop vieille ou si la corde est trop tendue. Le point "
        "est capture quand le segment de corde passe a moins de rayon_capture "
        "du point. Le retour est atteint quand le milieu revient a moins de "
        "rayon_arrivee du point de retour."
    )
    PARAMETRES = [
        _param("bouee_lat", "Bouee : latitude (deg)", 48.199072),
        _param("bouee_lon", "Bouee : longitude (deg)", -3.016258),
        _param("retour_mode", "Retour : mode", "depart", type="choix",
               choix=["depart", "point"], cacher=True),
        _param("retour_lat", "Retour : latitude (deg)", 48.198863),
        _param("retour_lon", "Retour : longitude (deg)", -3.015070),
        _param("longueur_corde", "Longueur de corde (m)", 10.0),
        _param("rayon_capture", "Rayon de capture (m)", 1.5),
        _param("rayon_arrivee", "Rayon d'arrivee au point de retour (m)", 6.0),
        _param("rayon_repulsion", "Rayon de repulsion (m)", 6.0),
        _param("k_repulsion", "Gain de repulsion", 0.4),
        _param("poids_repulsion", "Poids de la repulsion", 1.0),
        _param("rayon_urgence", "Arret d'urgence sous (m)", 4.0),
        _param("age_max", "Position de l'autre trop vieille apres (s)", 3.0),
        _param("vitesse", "Vitesse de formation (commande moteur)", 150,
               type="entier"),
        _param("k_virage", "Gain de virage (1/rad)", 0.7),
        _param("seuil_toutdroit_deg", "Seuil tout droit (deg)", 15.0),
        _param("timeout", "Duree maximale (s)", 1200.0),
        _param("attente", "Compte a rebours (s)", 10.0),
        _param("derive", "Compensation de derive", "oui", type="choix",
               choix=CHOIX_OUI_NON),
    ]

    ATTENTE_AUTRE_MAX = 60.0
    REPRISE_URGENCE = 3.0       # reprise apres l'arret d'urgence (m)
    # La position de l'autre et la notre arrivent a 1 Hz : a 1,5 m/s,
    # l'ecart estime peut etre faux de 1 a 3 m. On garde donc une marge
    # large avant de conclure que la corde est trop tendue.
    MARGE_CORDE = 4.0           # arret si d_autre > L + MARGE_CORDE
    MARGE_CORDE_REPRISE = 2.0   # reprise si d_autre <= L + MARGE_CORDE_REPRISE

    # ------------------------------------------------------------------
    # Parametres et apercu
    # ------------------------------------------------------------------
    def _verifier_parametres(self):
        p = self.p
        if abs(p["bouee_lat"]) > 90.0:
            return "latitude de bouee invalide"
        if abs(p["bouee_lon"]) > 180.0:
            return "longitude de bouee invalide"
        if p["retour_mode"] not in ("depart", "point"):
            return "mode de retour inconnu"
        if p["retour_mode"] == "point":
            if abs(p["retour_lat"]) > 90.0:
                return "latitude de retour invalide"
            if abs(p["retour_lon"]) > 180.0:
                return "longitude de retour invalide"
        if p["longueur_corde"] <= 0.0:
            return "longueur de corde > 0"
        if p["rayon_capture"] <= 0.0:
            return "rayon de capture > 0"
        if p["rayon_arrivee"] <= 0.0:
            return "rayon d'arrivee > 0"
        if p["rayon_urgence"] <= 0.0:
            return "rayon d'urgence > 0"
        if p["rayon_repulsion"] <= p["rayon_urgence"]:
            return "il faut rayon de repulsion > rayon d'urgence"
        if p["rayon_repulsion"] >= p["longueur_corde"]:
            return "le rayon de repulsion doit rester sous la longueur de corde"
        if p["k_repulsion"] < 0.0:
            return "gain de repulsion >= 0"
        if p["poids_repulsion"] < 0.0:
            return "poids de repulsion >= 0"
        if p["age_max"] <= 0.0 or p["timeout"] <= 0.0:
            return "age maximal et duree maximale doivent etre positifs"
        if p["vitesse"] <= 0:
            return "vitesse > 0"
        if p["k_virage"] <= 0.0:
            return "k_virage > 0"
        if not 0.0 <= p["seuil_toutdroit_deg"] < 90.0:
            return "seuil tout droit entre 0 et 90 deg"
        return None

    @staticmethod
    def _local(session, lat, lon):
        x, y = session.convertisseur.convert_cart_position(
            lat, lon, session.env.home_latlong, session.env.rotation)
        return np.array([float(x), float(y)])

    @classmethod
    def apercu(cls, session, parametres):
        """Aller (milieu -> point) et retour prevu, pour la carte."""
        bouee = cls._local(session, parametres["bouee_lat"],
                           parametres["bouee_lon"])
        retour = None
        if parametres["retour_mode"] == "point":
            retour = cls._local(session, parametres["retour_lat"],
                                parametres["retour_lon"])
        position = session.position_actuelle()
        autre = session.etat_autre()
        if position is not None and autre is not None:
            centre = 0.5 * (np.array([position[0], position[1]], dtype=float)
                            + np.array([autre["x"], autre["y"]], dtype=float))
            return [centre, bouee, centre if retour is None else retour]
        if retour is not None:
            return [bouee, retour]
        return [bouee]

    # ------------------------------------------------------------------
    # Petites aides
    # ------------------------------------------------------------------
    def _phase(self, phase, message=""):
        """Change la phase affichee seulement si elle differe."""
        if self.s.phase != phase:
            self.s.definir_phase(phase, message)
        elif message:
            self.s.definir_message(message)

    def _cap_deg(self):
        cap = self.ddb.dernier_cap
        return math.degrees(cap) if cap is not None else None

    @staticmethod
    def _ordonner(position, pos_autre, config):
        """Retourne (p14, p18) : 14 est le bateau au plus petit id.

        Les noms 14 et 18 sont ceux de la mission. Si les ids changent,
        le plus petit id joue le role de 14 et l'autre celui de 18.
        """
        position = np.asarray(position, dtype=float)
        pos_autre = np.asarray(pos_autre, dtype=float)
        if config["mon_id"] < config["autre_id"]:
            return position, pos_autre
        return pos_autre, position

    def _autre_frais(self):
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
        fin = time.monotonic() + self.ATTENTE_AUTRE_MAX
        while time.monotonic() < fin:
            s.verifier_arret()
            autre = self._autre_frais()
            if autre is not None:
                return autre
            time.sleep(0.2)
        s.ecrire_journal("pas_de_lien", {"attente": self.ATTENTE_AUTRE_MAX})
        s.definir_phase("ECHEC", "aucune position de l'autre bateau")
        return None

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------
    def executer(self):
        s = self.s
        erreur = self._verifier_parametres()
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
        configuration = s.configuration_communication()

        bouee = self._local(s, self.p["bouee_lat"], self.p["bouee_lon"])
        position, age = s.position_fraiche()
        if position is None:
            s.definir_phase("ECHEC", "aucun fix GPS")
            return 1, "pas_de_gps"
        autre = self._autre_frais()
        if autre is None:
            s.definir_phase("ECHEC", "position de l'autre perdue")
            return 1, "pas_de_lien"

        pos_autre = np.array([autre["x"], autre["y"]], dtype=float)
        p14, p18 = self._ordonner(position, pos_autre, configuration)
        centre_depart = 0.5 * (p14 + p18)
        if float(np.linalg.norm(centre_depart - bouee)) < self.p["rayon_capture"]:
            s.definir_phase("ECHEC", "le milieu de la corde est deja sur la bouee")
            return 1, "depart_sur_bouee"

        depart_latlon = s.local_vers_latlon(centre_depart[0], centre_depart[1])
        if self.p["retour_mode"] == "depart":
            centre_retour = centre_depart.copy()
            retour_latlon = depart_latlon
        else:
            centre_retour = self._local(s, self.p["retour_lat"],
                                        self.p["retour_lon"])
            retour_latlon = (self.p["retour_lat"], self.p["retour_lon"])

        s.ajouter_point_kml("Bouee", self.p["bouee_lat"], self.p["bouee_lon"],
                            "point a capturer", "bouee")
        s.ajouter_point_kml("Depart", depart_latlon[0], depart_latlon[1],
                            "milieu de la corde au lancement", "depart")
        s.ajouter_point_kml("Retour", retour_latlon[0], retour_latlon[1],
                            "point de retour demande", "retour")

        s.ecrire_journal("depart", {
            "bouee": bouee,
            "bouee_latlon": (self.p["bouee_lat"], self.p["bouee_lon"]),
            "retour_mode": self.p["retour_mode"],
            "retour_latlon": retour_latlon,
            "centre_retour": centre_retour,
            "p14": p14, "p18": p18, "centre": centre_depart,
            "dist_autre": float(np.linalg.norm(position - pos_autre)),
            "cap": self._cap_deg(), "age": age, "autre_age": autre["age"],
            "configuration": configuration,
            "longueur_corde": self.p["longueur_corde"],
            "rayon_capture": self.p["rayon_capture"],
            "rayon_arrivee": self.p["rayon_arrivee"],
        })

        s.definir_guidage(
            chemin=[[float(centre_depart[0]), float(centre_depart[1])],
                    [float(bouee[0]), float(bouee[1])],
                    [float(centre_retour[0]), float(centre_retour[1])]],
            a=[float(centre_depart[0]), float(centre_depart[1])],
            b=[float(bouee[0]), float(bouee[1])],
            zones=[self.p["rayon_capture"]],
            n_segments=2, derive=self.p["derive"] == "oui")

        s.definir_phase("DEPART", "depart dans {:.0f} s".format(self.p["attente"]))
        s.compte_a_rebours(self.p["attente"])
        return self._naviguer(bouee, centre_depart, centre_retour, configuration)

    def _naviguer(self, bouee, centre_depart, centre_retour, configuration):
        s, ddb, p = self.s, self.ddb, self.p
        derive = p["derive"] == "oui"
        ddb.reinitialiser_asservissement()

        debut = time.monotonic()
        phase = "APPROCHE"
        cible = bouee
        n_precedent = None
        u_aller = mc.vecteur_unitaire(bouee - centre_depart)
        capture_ok = False
        capture_manquee = False
        arrive = False
        issue = "termine"
        drapeaux = {"gps": False, "lien": False, "urgence": False, "corde": False}
        bilan = {
            "dist_min_autre": None, "dist_max_autre": 0.0,
            "capture_dist_min": None, "duree": 0.0,
            "capture": False, "capture_manquee": False,
            "resultat": None, "phase_finale": None,
            "bouee": bouee, "centre_depart": centre_depart,
            "centre_retour": centre_retour, "retour_mode": p["retour_mode"],
            "longueur_corde": p["longueur_corde"],
            "rayon_capture": p["rayon_capture"],
            "rayon_arrivee": p["rayon_arrivee"],
        }

        s.definir_phase("APPROCHE", "vers la bouee")
        try:
            while True:
                s.verifier_arret()
                maintenant = time.monotonic()
                if maintenant - debut > p["timeout"]:
                    ddb._arreter_moteurs()
                    s.ecrire_journal("timeout", {"duree": maintenant - debut})
                    issue = "timeout"
                    break

                position, age = s.position_fraiche()
                if position is None:
                    ddb._arreter_moteurs()
                    if not drapeaux["gps"]:
                        drapeaux["gps"] = True
                        s.ecrire_journal("gps_perime", {"age": age})
                        s.definir_message("GPS perdu, moteurs coupes")
                    time.sleep(0.2)
                    continue
                if drapeaux["gps"]:
                    drapeaux["gps"] = False
                    s.ecrire_journal("gps_retabli", {"age": age})
                    ddb.reinitialiser_asservissement()

                autre = self._autre_frais()
                if autre is None:
                    ddb._arreter_moteurs()
                    if not drapeaux["lien"]:
                        drapeaux["lien"] = True
                        brut = s.etat_autre()
                        s.ecrire_journal("lien_perdu", {
                            "age_autre": brut["age"] if brut else None,
                            "x": position[0], "y": position[1]})
                        s.definir_phase("LIEN_PERDU",
                                        "position de l'autre trop vieille ou absente")
                    time.sleep(0.1)
                    continue
                if drapeaux["lien"]:
                    drapeaux["lien"] = False
                    s.ecrire_journal("lien_retabli", {"age_autre": autre["age"]})
                    ddb.reinitialiser_asservissement()
                    self._phase(phase, "lien retabli")

                pos_autre = np.array([autre["x"], autre["y"]], dtype=float)
                p14, p18 = self._ordonner(position, pos_autre, configuration)
                centre = 0.5 * (p14 + p18)
                d_autre = float(np.linalg.norm(position - pos_autre))
                if bilan["dist_min_autre"] is None or d_autre < bilan["dist_min_autre"]:
                    bilan["dist_min_autre"] = d_autre
                if d_autre > bilan["dist_max_autre"]:
                    bilan["dist_max_autre"] = d_autre

                # --- securite : trop proche --------------------------------
                if d_autre < p["rayon_urgence"] or (
                        drapeaux["urgence"]
                        and d_autre < p["rayon_urgence"] + self.REPRISE_URGENCE):
                    ddb._arreter_moteurs()
                    if not drapeaux["urgence"]:
                        drapeaux["urgence"] = True
                        s.ecrire_journal("arret_urgence", {
                            "x": position[0], "y": position[1],
                            "dist_autre": d_autre, "age_autre": autre["age"]})
                        s.definir_phase("ARRET_URGENCE",
                                        "autre bateau a {:.1f} m".format(d_autre))
                    time.sleep(0.1)
                    continue
                if drapeaux["urgence"]:
                    drapeaux["urgence"] = False
                    s.ecrire_journal("reprise_urgence", {"dist_autre": d_autre})
                    ddb.reinitialiser_asservissement()
                    self._phase(phase, "reprise apres arret d'urgence")

                # --- securite : corde trop tendue -------------------------
                marge = (self.MARGE_CORDE_REPRISE if drapeaux["corde"]
                         else self.MARGE_CORDE)
                if d_autre > p["longueur_corde"] + marge:
                    ddb._arreter_moteurs()
                    if not drapeaux["corde"]:
                        drapeaux["corde"] = True
                        s.ecrire_journal("corde_tendue", {
                            "dist_autre": d_autre,
                            "longueur_corde": p["longueur_corde"]})
                        s.definir_phase("ARRET_URGENCE",
                                        "corde trop tendue ({:.1f} m)".format(d_autre))
                    time.sleep(0.1)
                    continue
                if drapeaux["corde"]:
                    drapeaux["corde"] = False
                    s.ecrire_journal("corde_retablie", {"dist_autre": d_autre})
                    ddb.reinitialiser_asservissement()
                    self._phase(phase, "corde retablie")

                # --- cible et loi de formation ----------------------------
                cible = bouee if phase == "APPROCHE" else centre_retour
                u = mc.vecteur_unitaire(cible - centre)
                n = mc.normale_formation(p14, p18, cible, n_precedent)
                if float(np.linalg.norm(n)) < 1e-6 or float(np.linalg.norm(u)) < 1e-6:
                    ddb._arreter_moteurs()
                    s.ecrire_journal("position_degeneree", {
                        "phase": phase,
                        "x": position[0], "y": position[1],
                        "centre_x": centre[0], "centre_y": centre[1]})
                    issue = "position_degeneree"
                    break

                n_precedent = n
                err = mc.erreur_cap(n, u)
                cote = mc.cote_bateau(position, centre, n)
                is_left = bool(cote > 0.0)
                v_i = mc.u_envitesse(
                    err, is_left, vmax=p["vitesse"], k_virage=p["k_virage"],
                    seuil_toutdroit=math.radians(p["seuil_toutdroit_deg"]))
                repulsion = mc.potentiel_repulsif(
                    pos_autre[0], pos_autre[1], position[0], position[1],
                    rayon=p["rayon_repulsion"], k=p["k_repulsion"])
                direction = mc.direction_bateau(n, repulsion, p["poids_repulsion"])

                cap_cible, cap, gauche, droite = ddb.maintien_cap(
                    direction, self.env.rotation, derive=derive, vitesse=v_i)
                corrige = mc.borner_marche_arriere(gauche, droite)
                if corrige != (gauche, droite):
                    s.ecrire_journal("double_marche_arriere", {
                        "commande": (gauche, droite), "corrigee": corrige})
                    gauche, droite = corrige
                    ddb._commander_moteurs(gauche, droite)

                # --- mesures de capture et d'arrivee ----------------------
                capture = False
                d_seg = None
                t_seg = None
                d_cible = float(np.linalg.norm(centre - cible))
                dist_depart = float(np.linalg.norm(centre - centre_depart))
                dist_retour = float(np.linalg.norm(centre - centre_retour))
                if phase == "APPROCHE":
                    capture, d_seg, t_seg = mc.capture_bouee(
                        bouee, p14, p18, p["rayon_capture"])
                    if (bilan["capture_dist_min"] is None
                            or d_seg < bilan["capture_dist_min"]):
                        bilan["capture_dist_min"] = d_seg

                asserv = ddb.dernier_asservissement or {}
                sol = ddb.vitesse_sol or {}
                s.ecrire_journal("nav", {
                    "phase": phase,
                    "x": position[0], "y": position[1], "age": age,
                    "cap": self._cap_deg(),
                    "autre_x": pos_autre[0], "autre_y": pos_autre[1],
                    "autre_cap": autre["cap"], "age_autre": autre["age"],
                    "dist_autre": d_autre,
                    "centre_x": centre[0], "centre_y": centre[1],
                    "cible_x": cible[0], "cible_y": cible[1],
                    "dist_cible": d_cible, "dist_depart": dist_depart,
                    "retour_x": centre_retour[0], "retour_y": centre_retour[1],
                    "dist_retour": dist_retour,
                    "n": n, "u": u,
                    "err": err, "err_deg": math.degrees(err),
                    "cote": cote, "is_left": is_left, "v_i": v_i,
                    "repulsion": repulsion,
                    "rep_norme": float(np.linalg.norm(repulsion)),
                    "direction": direction,
                    "cap_cible": math.degrees(cap_cible),
                    "cap_effectif": math.degrees(cap),
                    "erreur_cap": math.degrees(angle_diff(cap_cible, cap)),
                    "moteurs": (gauche, droite),
                    "capture": capture, "capture_dist": d_seg,
                    "capture_t": t_seg,
                    "v_sol": sol.get("vitesse"), "cog": sol.get("cog"),
                    "asserv": asserv,
                })
                s.definir_guidage(
                    dist_cible=d_cible,
                    direction=[float(direction[0]), float(direction[1])],
                    route=asserv.get("route"), crabe=asserv.get("crabe"))
                self._phase(phase, "cible a {:.1f} m".format(d_cible))

                # --- capture, bouee manquee, retour ----------------------
                if phase == "APPROCHE":
                    if capture:
                        capture_ok = True
                        cap_lat, cap_lon = s.local_vers_latlon(
                            centre[0], centre[1])
                        s.ajouter_point_kml("Capture", cap_lat, cap_lon,
                                            "capture effective", "capture")
                        s.ecrire_journal("capture", {
                            "x": position[0], "y": position[1],
                            "centre_x": centre[0], "centre_y": centre[1],
                            "dist_seg": d_seg, "capture_t": t_seg,
                            "dist_autre": d_autre, "age_autre": autre["age"]})
                        phase = "RETOUR"
                        cible = centre_retour
                        s.definir_phase("RETOUR", "bouee capturee, retour")
                        s.definir_guidage(
                            a=[float(centre[0]), float(centre[1])],
                            b=[float(centre_retour[0]), float(centre_retour[1])],
                            zones=[p["rayon_arrivee"]])
                        ddb.reinitialiser_asservissement()
                    else:
                        avance = float(np.dot(centre - bouee, u_aller))
                        if (avance > p["rayon_capture"]
                                and d_cible > p["rayon_capture"] + 3.0):
                            capture_manquee = True
                            s.ecrire_journal("capture_manquee", {
                                "x": position[0], "y": position[1],
                                "centre_x": centre[0], "centre_y": centre[1],
                                "dist_cible": d_cible, "dist_seg": d_seg,
                                "capture_t": t_seg, "avance": avance})
                            phase = "RETOUR"
                            cible = centre_retour
                            s.definir_phase("RETOUR",
                                            "bouee manquee, retour")
                            s.definir_guidage(
                                a=[float(centre[0]), float(centre[1])],
                                b=[float(centre_retour[0]), float(centre_retour[1])],
                                zones=[p["rayon_arrivee"]])
                            ddb.reinitialiser_asservissement()
                elif phase == "RETOUR":
                    if dist_retour < p["rayon_arrivee"]:
                        ddb._arreter_moteurs()
                        s.ecrire_journal("arrivee", {
                            "x": position[0], "y": position[1],
                            "centre_x": centre[0], "centre_y": centre[1],
                            "dist_retour": dist_retour,
                            "dist_depart": dist_depart,
                            "dist_autre": d_autre})
                        arrive = True
                        break

                time.sleep(0.1)

        except ArretDemande:
            issue = "arret"
            raise
        except BaseException:
            issue = "erreur"
            raise
        finally:
            try:
                ddb._arreter_moteurs()
            except Exception:
                pass
            bilan["duree"] = time.monotonic() - debut
            bilan["capture"] = capture_ok
            bilan["capture_manquee"] = capture_manquee
            if arrive:
                bilan["resultat"] = ("capture_manquee" if capture_manquee
                                     else "termine")
            else:
                bilan["resultat"] = (issue if issue != "termine"
                                     else "interrompu")
            bilan["phase_finale"] = s.phase
            s.ecrire_journal("bilan", bilan)
            s.definir_guidage()

        if (not arrive) or issue != "termine":
            s.definir_phase("ECHEC", "mission interrompue ({})".format(issue))
            return 1, issue
        if capture_manquee:
            s.definir_phase("ECHEC", "bouee manquee, retour effectue")
            return 1, "capture_manquee"
        s.definir_phase("TERMINE", "retour effectue")
        return 0, "termine"
