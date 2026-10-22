# -*- coding: utf-8 -*-
"""Mission capture : premiere couche, la formation.

Deux ddboats sont relies par une corde. On modelise l'ensemble comme un
bateau virtuel (la formation) dont les deux "moteurs" sont les deux
ddboats :

    formation : etat (C, n), commande -> une vitesse par bateau
    bateau i  : etat (x_i, y_i, theta_i), commandes (V_i, u_i)

C est le milieu de la corde, n la normale a la corde orientee vers la
cible (le point a capturer a l'aller, le point de retour choisi). La
formation avance vers la cible et tourne en faisant avancer un bateau
plus que l'autre. Le signe de n est garde par continuite : au retour, la
cible change, mais la normale ne se retourne pas instantanement ; c'est
l'erreur n -> u qui fait faire le demi-tour.

La commande est volontairement simple : un gain proportionnel sur
l'erreur de cap. Chaque bateau calcule la meme erreur, regarde de quel
cote de la formation il se trouve, et en deduit sa vitesse cible avec
u_envitesse(). L'autre bateau fait le meme calcul avec l'autre cote.

Conventions
-----------
- Repere local du lac : x vers l'est, y vers le nord.
- Un vecteur est un tableau numpy [x, y].
- R90(v) = (-v_y, v_x) est la rotation de 90 degres vers la gauche.
- Les angles sont en radians, dans le sens trigonometrique.
- err = erreur_cap(n, u) est positive si la cible u est a DROITE de n
  (convention "cap" classique : positif = tourner a droite).
- Les vitesses sont en unites de commande moteur (0 a 255 environ).
- is_left = True pour le bateau a gauche de la formation, False a droite.

Attention aux unites avec le reseau : communication.py echange le cap en
DEGRES (0 a 360). Ici, erreur_cap() travaille sur des vecteurs et renvoie
des RADIANS. Si tu utilises le cap recu de l'autre bateau dans un calcul
d'angle, convertis-le avec math.radians() avant. Le gain K_VIRAGE et le
seuil SEUIL_TOUTDROIT sont exprimes pour des radians.
"""

import math

import numpy as np

EPS = 1e-9

# Valeurs par defaut de la commande de formation. Le wrapper passera
# normalement VMAX = vitesse de la mission.
VMAX = 150                  # commande moteur maximale (a ajuster)
K_VIRAGE = 0.7              # gain proportionnel, en 1/radian
SEUIL_TOUTDROIT_DEG = 15.0  # zone morte, en degres (lisible)
SEUIL_TOUTDROIT = math.radians(SEUIL_TOUTDROIT_DEG)  # en radians

# Plancher du ralentissement pres de la bouee.
# 1.0 = pas de ralentissement en fin de mission (choix actuel).
# Remettre par exemple 0.3 pour ralentir pres de la bouee.
FACTEUR_ARRIVEE_MIN = 1.0


# ----------------------------------------------------------------------
# Geometrie de base
# ----------------------------------------------------------------------
def vecteur_unitaire(v):
    """Retourne v / |v|, ou (0, 0) si |v| est trop petit.

    Exemple : vecteur_unitaire([3, 4]) -> [0.6, 0.8].
    """
    v = np.asarray(v, dtype=float)
    norme = float(math.hypot(v[0], v[1]))
    if norme < EPS:
        return np.zeros(2)
    return v / norme


def perpendiculaire_gauche(v):
    """Retourne la normale gauche de v : (-v_y, v_x).

    Exemple : v = [1, 0] (est) -> [0, 1] (nord).
    C'est une rotation de 90 degres dans le sens trigonometrique.
    """
    v = np.asarray(v, dtype=float)
    return np.array([-v[1], v[0]])


def vecteur_normal(a, b):
    """Normale gauche unitaire du segment dirige de a vers b.

    C'est la perpendiculaire au vecteur (b - a), tournee vers la gauche.
    Il y a deux normales possibles ; celle-ci est la normale gauche.
    Pour choisir celle qui pointe vers la bouee, voir normale_formation().

    Exemple : a = (0, 0), b = (0, 1) (segment vers le nord)
              -> normale gauche = (-1, 0) (ouest).
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return perpendiculaire_gauche(vecteur_unitaire(b - a))


def angle_signe(a, b):
    """Angle signe de a vers b, en radians, dans ]-pi, pi].

    Positif si b est a gauche de a (sens trigonometrique), negatif sinon.
    C'est un angle mathematique, pas un cap boussole.

    Formule : atan2(a x b, a . b), ou "x" est le produit vectoriel 2D
    a_x b_y - a_y b_x. atan2 donne directement le bon quadrant et un
    angle dans ]-pi, pi].

    Exemple : a = (1, 0) (est), b = (0, 1) (nord) -> +pi/2.
              a = (1, 0), b = (0, -1) (sud)           -> -pi/2.
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    produit_vectoriel = a[0] * b[1] - a[1] * b[0]
    produit_scalaire = a[0] * b[0] + a[1] * b[1]
    if abs(produit_vectoriel) < EPS and abs(produit_scalaire) < EPS:
        return 0.0
    return float(math.atan2(produit_vectoriel, produit_scalaire))


# ----------------------------------------------------------------------
# Formation : normale, erreur, cote, commande
# ----------------------------------------------------------------------
def normale_formation(p14, p18, cible=None, n_precedent=None):
    """Normale a la corde, avec un signe continu.

    p14 et p18 sont les positions des deux bateaux. La corde va de 14
    vers 18. On prend la normale gauche de ce segment, puis on choisit
    son signe :

    - si n_precedent est fourni, on garde le signe le plus proche de la
      normale precedente. C'est le cas normal en boucle : la normale est
      un etat de la formation, elle ne doit pas se retourner toute seule.
    - sinon, on choisit le cote de la cible (point a capturer ou point
      de retour).

    Pourquoi la continuite est importante : au retour, la cible passe de
    la bouee au point de depart. Si on rechoisissait le signe vers la
    nouvelle cible, n se retournerait instantanement et l'erreur serait
    nulle : la formation croirait etre alignee alors que les bateaux
    regardent encore dans l'autre sens. Avec n_precedent, n reste le
    meme, l'erreur vaut 180 deg, et la formation fait son demi-tour.

    Retour : vecteur unitaire n, ou (0, 0) si les bateaux sont confondus.
    """
    p14 = np.asarray(p14, dtype=float)
    p18 = np.asarray(p18, dtype=float)
    n = vecteur_normal(p14, p18)
    if float(np.linalg.norm(n)) < EPS:
        return np.zeros(2)

    if n_precedent is not None:
        n_precedent = np.asarray(n_precedent, dtype=float)
        if float(np.linalg.norm(n_precedent)) > EPS:
            if float(np.dot(n, n_precedent)) < 0.0:
                n = -n
            return n

    if cible is None:
        return n

    centre = 0.5 * (p14 + p18)
    u = vecteur_unitaire(np.asarray(cible, dtype=float) - centre)
    if float(np.linalg.norm(u)) < EPS:
        return np.zeros(2)
    if float(np.dot(n, u)) < 0.0:
        n = -n
    return n


def erreur_cap(n, u):
    """Erreur de cap : positive si la cible u est a DROITE de la normale n.

    C'est la convention des caps : err > 0 veut dire "tourner a droite",
    err < 0 "tourner a gauche", err = 0 "tout droit".

    Pourquoi angle_signe(u, n) et pas angle_signe(n, u) ?
    angle_signe(a, b) est positif si b est a gauche de a.
    - angle_signe(n, u) > 0 signifie u a gauche de n -> positif a gauche.
    - angle_signe(u, n) > 0 signifie n a gauche de u, donc u a droite
      de n -> positif a droite. C'est la convention de u_envitesse().

    Exemple : n = (1, 0) (est), u = (0, 1) (nord)
              -> la cible est a gauche -> err = -pi/2.
              n = (1, 0), u = (0, -1) (sud)
              -> la cible est a droite -> err = +pi/2.
    """
    return angle_signe(u, n)


def cote_bateau(position, centre, normale):
    """Cote d'un bateau par rapport a l'axe de la formation.

    Retourne +1 s'il est a gauche de la formation, -1 s'il est a droite.
    "Gauche" signifie du cote de R90(normale) : si la formation regarde
    vers l'est (n = est), la gauche est le nord.

    Les deux bateaux doivent avoir des cotes opposes. En pratique,
    position - centre vaut +/- (L/2) * t, et R90(normale) vaut +/- t ;
    le produit scalaire vaut donc +/- L/2, jamais 0.

    Exemple : 14 au sud, 18 au nord, n vers l'est
              -> cote(14) = -1 (droite), cote(18) = +1 (gauche).

    Le wrapper utilise is_left = (cote_bateau(...) > 0). Il doit
    verifier que les deux bateaux ont bien des cotes opposes. S'il veut
    empecher les bateaux de se croiser, il peut garder le cote du
    depart ; dans ce cas, il doit aussi garder le signe de la normale
    (hysteresis) pour que les deux restent coherents.
    """
    position = np.asarray(position, dtype=float)
    centre = np.asarray(centre, dtype=float)
    normale = np.asarray(normale, dtype=float)
    gauche = perpendiculaire_gauche(normale)
    if float(np.dot(position - centre, gauche)) >= 0.0:
        return 1.0
    return -1.0


def u_envitesse(err, is_left, vmax=VMAX, k_virage=K_VIRAGE,
                seuil_toutdroit=SEUIL_TOUTDROIT):
    """Vitesse cible d'un bateau, cote gauche ou droit de la formation.

    C'est le gain proportionnel classique :

        err > 0 : la cible est a droite -> le cote droit ralentit ;
        err < 0 : la cible est a gauche -> le cote gauche ralentit ;
        |err| <= seuil : tout droit, les deux cotes a vmax.

    Le cote qui ne ralentit pas reste a vmax. Ainsi aucune vitesse n'est
    negative et le bateau interieur peut s'arreter (k sature a 1), mais
    ne recule jamais. Le rayon de courbure minimal de la formation est
    alors L/2 (un cote a vmax, l'autre a 0).

    Entrees
    -------
    err : erreur de cap en radians, positive a droite (voir erreur_cap).
    is_left : True si ce bateau est a gauche de la formation.
    vmax : vitesse du cote qui ne ralentit pas (commande moteur).
    k_virage : gain proportionnel, en 1/radian.
    seuil_toutdroit : zone morte en radians.

    Sortie
    ------
    La vitesse cible de ce bateau, en commande moteur (entier).

    Exemple : err = -45 deg (cible a gauche), is_left = True, vmax = 120,
              k_virage = 0.7 -> k = -0.55 -> v = 120 * (1 - 0.55) = 54.
              Le cote gauche ralentit, la formation tourne a gauche.

    Attention : err est en radians. Le cap echange par communication.py
    est en degres (0 a 360) : convertis-le avec math.radians() avant de
    calculer err. Si tu calcules l'erreur en degres, convertis-la aussi.
    """
    vmax = int(max(0, vmax))
    if abs(err) <= seuil_toutdroit:
        return vmax
    k = max(-1.0, min(1.0, k_virage * err))
    if k >= 0:
        # Cible a droite : le cote droit ralentit, le gauche reste a vmax.
        if is_left:
            return vmax
        return int(vmax * (1.0 - k))
    # Cible a gauche : le cote gauche ralentit, le droit reste a vmax.
    if is_left:
        return int(vmax * (1.0 + k))
    return vmax


def facteur_arrivee(distance, distance_ralentir):
    """Facteur de vitesse qui ralentit la formation pres de la bouee.

    distance >= distance_ralentir -> 1.0
    distance = 0                 -> FACTEUR_ARRIVEE_MIN
    entre les deux               -> interpolation lineaire

    FACTEUR_ARRIVEE_MIN vaut 1.0 actuellement : le facteur reste 1.0,
    donc pas de ralentissement en fin de mission. Remettre par exemple
    0.3 pour ralentir pres de la bouee. On ne met jamais 0 : la
    formation doit continuer a avancer jusqu'a la capture ; c'est le
    test de capture qui coupe les moteurs.

    Le wrapper fait :
        vmax_bouee = VMAX * facteur_arrivee(distance, distance_ralentir)
        v_i = u_envitesse(err, is_left, vmax=vmax_bouee)
    """
    if distance_ralentir <= 0.0:
        return 1.0
    return max(FACTEUR_ARRIVEE_MIN,
               min(1.0, float(distance) / float(distance_ralentir)))


# ----------------------------------------------------------------------
# Niveau bateau : direction, repulsion, capture
# ----------------------------------------------------------------------
def potentiel_repulsif(x_pt, y_pt, x_pos, y_pos, rayon=2, k=1):
    """Repulsion de l'autre bateau, version discontinue (choix actuel).

    (x_pt, y_pt) est l'autre bateau, (x_pos, y_pos) est notre position.
    Le vecteur pointe de l'autre bateau vers nous, donc il nous ecarte.
    Nul au-dela de rayon ; a l'interieur, norme constante k.

    La discontinuite au rayon est voulue pour l'instant : le bateau prend
    une correction d'un coup en entrant dans la zone. On la lissera plus
    tard si les essais montrent un a-coup.

    Garde : si les deux positions sont confondues, on renvoie (0, 0)
    au lieu de diviser par zero.
    """
    dist = np.sqrt((x_pt - x_pos) ** 2 + (y_pt - y_pos) ** 2)
    if dist > rayon:
        return 0
    direction = np.array([x_pos - x_pt, y_pos - y_pt])
    norme = float(np.linalg.norm(direction))
    if norme < EPS:
        return np.zeros(2)
    direction /= norme
    return k * direction


def direction_bateau(normale, repulsion, poids_repulsion=1.0):
    """Direction commandee a un bateau : la normale corrigee par la repulsion.

    normale : n, direction de la formation (vecteur unitaire).
    repulsion : vecteur de potentiel_repulsif (ou 0).
    poids_repulsion : poids de la correction de securite.

    Le bateau suit n. Il ne vise jamais la bouee : c'est la formation
    qui gere la cible. La repulsion sert seulement a ne pas trop se
    rapprocher de l'autre bateau.

    Si n et la repulsion s'annulent exactement, on garde n pour ne pas
    envoyer une direction nulle a maintien_cap().
    """
    direction = (np.asarray(normale, dtype=float)
                 + float(poids_repulsion) * np.asarray(repulsion, dtype=float))
    if float(np.linalg.norm(direction)) < EPS:
        return vecteur_unitaire(normale)
    return vecteur_unitaire(direction)


def distance_point_segment(point, a, b):
    """Distance d'un point au segment [a, b] et parametre de projection.

    Retourne (distance, t) :
      distance : distance du point au segment (m) ;
      t : position de la projection sur la droite (a, b).
          t = 0 sur a, t = 1 sur b, 0 < t < 1 entre les deux.
          t peut sortir de [0, 1] si la projection est au-dela de a ou b.

    La distance, elle, est toujours celle du segment : si la projection
    sort de [0, 1], on prend la distance au bout le plus proche.
    """
    point = np.asarray(point, dtype=float)
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    ab = b - a
    longueur2 = float(np.dot(ab, ab))
    if longueur2 < EPS * EPS:
        return float(np.linalg.norm(point - a)), 0.0
    t = float(np.dot(point - a, ab) / longueur2)
    t_segment = max(0.0, min(1.0, t))
    proj = a + t_segment * ab
    return float(np.linalg.norm(point - proj)), t


def capture_bouee(bouee, p14, p18, rayon):
    """Teste si la corde attrape la bouee.

    La corde est le segment [p14, p18]. On regarde la distance de la
    bouee a ce segment. Il y a capture si :
      - la distance est inferieure a rayon ;
      - la projection tombe entre les deux bateaux (0 <= t <= 1),
        c'est-a-dire si la bouee est bien entre eux, pas au-dela d'un
        bout de la corde.

    Retourne (capture, distance, t).
    """
    distance, t = distance_point_segment(bouee, p14, p18)
    capture = bool(distance <= rayon and 0.0 <= t <= 1.0)
    return capture, distance, t


def borner_marche_arriere(gauche, droite):
    """Un seul moteur en arriere : si les deux le sont, le moins negatif passe a 0.

    C'est la regle des missions precedentes. La vitesse de formation,
    elle, est deja positive ou nulle : u_envitesse() ne renvoie jamais
    une vitesse negative. Cette fonction protege le niveau moteur.
    """
    if gauche < 0 and droite < 0:
        if gauche >= droite:
            return 0, droite
        return gauche, 0
    return gauche, droite


# ----------------------------------------------------------------------
# Petit exemple executable : python3 mission_capture.py
# ----------------------------------------------------------------------
if __name__ == "__main__":
    # Les deux bateaux sont au sud et au nord du milieu, la corde est
    # verticale. On regarde trois positions de la bouee.
    p14 = np.array([0.0, -5.0])
    p18 = np.array([0.0, 5.0])
    centre = 0.5 * (p14 + p18)

    exemples = [
        ("aligne (bouee a l'est)", np.array([10.0, 0.0])),
        ("cible a gauche (nord-est)", np.array([10.0, 10.0])),
        ("cible a droite (nord-ouest)", np.array([-10.0, 10.0])),
    ]
    for nom, bouee in exemples:
        n = normale_formation(p14, p18, bouee)
        u = vecteur_unitaire(bouee - centre)
        err = erreur_cap(n, u)
        cote14 = cote_bateau(p14, centre, n)
        cote18 = cote_bateau(p18, centre, n)
        v14 = u_envitesse(err, cote14 > 0, vmax=VMAX)
        v18 = u_envitesse(err, cote18 > 0, vmax=VMAX)
        print("{} :".format(nom))
        print("  n = {}, err = {:.1f} deg".format(n, math.degrees(err)))
        print("  14 : cote = {:.0f} (gauche = {}) -> v = {}".format(
            cote14, cote14 > 0, v14))
        print("  18 : cote = {:.0f} (gauche = {}) -> v = {}".format(
            cote18, cote18 > 0, v18))
