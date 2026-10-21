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
| Mission D1 | Aller vers D1, tour complet, retour par le couloir |

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
- `mission_d1.py`, `missions.py` : les missions et essais ;
- `drivers/ddboatclass.py` : régulateur de cap (PID) et compensation de dérive ;
- `drivers/derive.py` : estimation de la dérive à partir du GPS ;
- `drivers/navigation.py` : gradient de ligne et gradient ponctuel ;
- `simulation.py` : bateau simulé.
