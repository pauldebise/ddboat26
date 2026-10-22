# ddboat26

Projet DD-Boat (géofencing, calibration magnétomètre, pilotage).

Voir [CONTRIBUTING.md](CONTRIBUTING.md) pour le workflow Git et les règles de
collaboration.

## IHM, essais et missions

Tout se lance depuis un seul programme, sur le bateau :

```bash
python3 ihm_lac.py --bateau
```

Ouvrir ensuite `http://<IP_BATEAU>:8000/` : le panneau **Missions** permet de
choisir un essai ou une mission, de régler ses paramètres, de **LANCER** et
d'**ARRÊTER** (le bouton ARRÊT coupe les moteurs immédiatement). Le chemin
prévu s'affiche sur la carte avant le lancement. Ctrl+C dans le terminal coupe
aussi les moteurs.

| Mission | Rôle |
|---|---|
| Essai 1 : caps fixes | Tient N, E, S, O : vérifie le suivi de cap, mesure le biais du compas et la dérive |
| Essai 2 : suivi de segment | Aller-retour sur un segment, sans puis avec compensation de dérive : mesure l'écart latéral |
| Essai 3 : ralliement d'un point | Gradient ponctuel seul, comme le homing |
| Mission : polygone | Parcourt le polygone de `polygone.json` (gradient de ligne + gradient ponctuel) |
| Mission 3 | Trajet A-C-D-G'-E-F-G-A de `mission3.json`, termine à A en ralentissant pour s'arrêter avant le ponton |
| Station keeping | Reste autour du point G, sans fin : moteurs coupés dans la grande zone (5 m), retour lent sans marche arrière dans la petite (2 m) |
| Mission : rejoindre un point | Rappelle le bateau vers un point connu de `points.json` ou choisi à la souris, puis s'arrête : voir ci-dessous |
| Mission ABBA | Rejoint le point de départ de l'autre bateau en l'évitant (positions échangées par le serveur UDP du lac) : voir ci-dessous |
| Mission capture | Aller-retour du milieu d'une corde entre deux bateaux : capture un point choisi, puis revient au départ, à un point connu ou à un point cliqué : voir ci-dessous |
| Mission D1 | Aller vers D1, tour complet, retour par le couloir |

### Mission : rejoindre un point

Cette mission ramène le bateau vers un point précis, par exemple le ponton ou
votre position au bord de l'eau, quand il est arrêté au milieu du lac. Dans le
panneau **Missions**, la ligne **Point** propose les points connus de
`points.json` et le bouton **Clic carte**. Le bateau rejoint le point avec le
gradient ponctuel, compense la dérive, ralentit sur les derniers mètres
(`ralentir_fin`, `vitesse_fin`) puis coupe les moteurs dans le rayon
d'arrivée (`rayon_arrivee`, 3 m par défaut). La mission se termine à
l'arrivée ; pour maintenir une position, utiliser ensuite le station keeping.

Elle ne dépend pas de l'autre bateau : seul le GPS est nécessaire. En ligne de
commande :

```bash
python3 ihm_lac.py --mission ralliement \
  --param cible_lat=48.198863 --param cible_lon=-3.015070
```

### Mission ABBA

Deux bateaux échangent leur position à 1 Hz par le serveur UDP du lac
(`communication.py`) et rejoignent chacun le point de départ de l'autre. Le
panneau **Communication** de l'IHM règle l'IP du serveur, le port, l'id du
bateau et celui de l'autre (enregistrés dans `communication.json`, propre à
chaque bateau). La communication tourne dès le démarrage : l'autre bateau, son
âge, le cercle de 15 m et l'état du lien s'affichent même en veille.

Avec `points = auto`, le départ est ma position GPS au lancement et l'arrivée la position de l'autre bateau à cet instant. Sinon, départ et arrivée se choisissent dans les listes (points de `points.json`) ou
d'un clic sur la carte (bouton **Clic carte**). Pendant la mission : répulsion
de l'autre bateau dans 15 m, ralentissement, arrêt d'urgence sous 5 m (reprise
au-delà de 8 m), arrêt si sa position a plus de 3 s ou est absente, attente à
10 m du but s'il est occupé. Journal : événements `evitement_debut/fin`,
`arret_urgence`, `lien_perdu`, `but_occupe`, `arrivee`, `bilan`.

`python3 test_fantome.py` rejoue la mission contre un bateau fantôme, sans
matériel (voir l'en-tête du script pour les scénarios).

### Mission capture

Deux ddboats sont reliés par une corde. La mission fait l'aller-retour du
milieu de la corde : le milieu va capturer le point choisi, puis revient à
son point de départ, à un point connu de `points.json`, ou à un point cliqué
sur la carte. Les deux bateaux lancent la même mission avec le même point de
capture et des ids différents (14 et 18) ; le panneau **Communication** doit
être configuré sur les deux. Dans le panneau **Missions**, la ligne **Bouée**
choisit le point à capturer parmi les points connus (ou **Clic carte**) ; la
ligne **Retour** propose **Départ de la mission (milieu)**, les points connus
et **Clic carte**. En ligne de commande, les mêmes choix passent par
`--param retour_mode=depart|point --param retour_lat=... --param retour_lon=...`.
Les trajectoires du bateau et de l'autre bateau sont tracées sur la carte (la
trace de l'autre est en violet).

La formation est commandée par un gain proportionnel : `n` est la normale à
la corde orientée vers la cible, `err` l'écart entre `n` et la direction de la
cible (positif à droite), et `u_envitesse()` ralentit le côté intérieur. Avec
un seuil de 15°, les deux bateaux vont tout droit ; au-delà, le côté intérieur
ralentit. Les bateaux ne reculent jamais : le côté intérieur peut s'arrêter,
ce qui donne un rayon de courbure minimal de `L/2`. La répulsion de l'autre
bateau s'ajoute à la direction (rayon 6 m, gain 0,4 par défaut), et les sécurités
coupent les moteurs si l'autre est trop proche (< 4 m), si sa position a plus
de 3 s, ou si la corde est trop tendue (> L + 4 m ; marge large car les
positions arrivent à 1 Hz). La capture est détectée quand le segment de corde
passe à moins de `rayon_capture` du point à capturer ; le retour est terminé
quand le milieu revient à moins de `rayon_arrivee` du point de retour (6 m par
défaut : avec L = 10 m et sans marche arrière, le rayon de courbure minimal
est 5 m, donc un rayon d'arrivée de 3 m ferait orbiter la formation).

Journaux : `depart`, `nav` (toutes les 0,1 s), `capture`, `capture_manquee`,
`arrivee`, `lien_perdu`, `arret_urgence`, `corde_tendue`, `bilan`, `kml`.
La loi est dans `mission_capture.py` ; le wrapper IHM est dans
`mission_capture_ihm.py`. À la fin de chaque mission, un fichier KML du même
nom que le journal est écrit dans `logs/` : trace du bateau, trace de l'autre
bateau, point à capturer, départ, retour et capture. Il est aussi
téléchargeable depuis l'IHM (`/kml`).

Le polygone se décrit dans `polygone.json` (voir `polygone.exemple.json`),
en coordonnées GPS (`points_latlon`) ou en mètres (`points_xy`).

Chaque mission écrit `logs/<mission>_<date>.jsonl` (une ligne JSON par
événement), téléchargeable sur `http://<IP_BATEAU>:8000/journal` ou avec
`scp ue32@<IP_BATEAU>:~/guerledan_s3/logs/*.jsonl .`. Le KML correspondant
est écrit à côté (`logs/<mission>_<date>.kml`) et téléchargeable sur
`http://<IP_BATEAU>:8000/kml` ; pour tout récupérer :
`scp ue32@<IP_BATEAU>:~/guerledan_s3/logs/*.jsonl ue32@<IP_BATEAU>:~/guerledan_s3/logs/*.kml .`.

Sans bateau, `python3 ihm_lac.py --bateau --simulation --acceleration 5`
lance le même programme sur un bateau simulé (vent, moteurs inégaux, compas
biaisé), avec ses journaux dans `logs/simulation/`.

Organisation du code :

- `session.py` : matériel, état partagé avec l'IHM, journal, exécution des missions ;
- `mission_d1.py`, `mission_communication.py`, `missions.py` : les missions et essais ;
- `mission_capture.py` : loi de la formation (normale, gain, répulsion, capture) ;
- `mission_capture_ihm.py` : wrapper de la mission capture pour l'IHM ;
- `drivers/ddboatclass.py` : régulateur de cap (PID) et compensation de dérive ;
- `drivers/derive.py` : estimation de la dérive à partir du GPS ;
- `drivers/navigation.py` : gradient de ligne et gradient ponctuel ;
- `simulation.py` : bateau simulé.
