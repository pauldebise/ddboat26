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
| Mission ABBA | Rejoint le point de départ de l'autre bateau en l'évitant (positions échangées par le serveur UDP du lac) : voir ci-dessous |
| Mission D1 | Aller vers D1, tour complet, retour par le couloir |

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

Le polygone se décrit dans `polygone.json` (voir `polygone.exemple.json`),
en coordonnées GPS (`points_latlon`) ou en mètres (`points_xy`).

Chaque mission écrit `logs/<mission>_<date>.jsonl` (une ligne JSON par
événement), téléchargeable sur `http://<IP_BATEAU>:8000/journal` ou avec
`scp ue32@<IP_BATEAU>:~/guerledan_s3/logs/*.jsonl .`.

Sans bateau, `python3 ihm_lac.py --bateau --simulation --acceleration 5`
lance le même programme sur un bateau simulé (vent, moteurs inégaux, compas
biaisé), avec ses journaux dans `logs/simulation/`.

Organisation du code :

- `session.py` : matériel, état partagé avec l'IHM, journal, exécution des missions ;
- `mission_d1.py`, `mission_communication.py`, `missions.py` : les missions et essais ;
- `drivers/ddboatclass.py` : régulateur de cap (PID) et compensation de dérive ;
- `drivers/derive.py` : estimation de la dérive à partir du GPS ;
- `drivers/navigation.py` : gradient de ligne et gradient ponctuel ;
- `simulation.py` : bateau simulé.
