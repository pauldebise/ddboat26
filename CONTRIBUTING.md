# Contribuer à ddboat26

## Règle d'or

`main` est la version stable : **personne ne pousse directement dessus**.
Chacun travaille sur une branche à son nom, puis ouvre une Pull Request.

## Configuration (une seule fois par machine)

```bash
git clone https://github.com/pauldebise/ddboat26.git
cd ddboat26
git config pull.rebase true      # un pull rejoue vos commits, sans commit de merge
git config rebase.autoStash true # met de côté les modifications non commitées pendant un rebase
git config fetch.prune true      # nettoie les branches distantes supprimées
```

## Cycle de travail quotidien

```bash
# 1. Partir d'un main à jour
git switch main
git pull --rebase

# 2. Créer sa branche (préfixe = prénom)
git switch -c paul/gradient-artificiel
# ou
git switch -c romeo/calibration

# 3. Coder et commiter petit et souvent
git add -p
git commit -m "ajoute le gradient artificiel ponctuel"

# 4. Se remettre à jour avant de pousser
git fetch origin
git rebase origin/main

# 5. Publier sa branche
git push -u origin paul/gradient-artificiel

# 6. Ouvrir une Pull Request sur GitHub, demander une relecture,
#    puis merger (Squash and merge ou Merge commit).

# 7. Après le merge, nettoyer
git switch main
git pull --rebase
git branch -d paul/gradient-artificiel
```

## En cas de conflit

```bash
git status                  # liste les fichiers en conflit
# Éditer les fichiers, supprimer les marqueurs <<<<<<< ======= >>>>>>>
git add fichier_resolu.py
git rebase --continue

# Pour tout annuler et revenir à l'état d'avant :
git rebase --abort
```

## Règles de collaboration

- **Jamais de commit direct sur `main`**, jamais de `force-push` sur `main`.
- Un `force-push` est autorisé uniquement sur **sa propre branche**, avec
  `git push --force-with-lease` (jamais `--force` seul).
- Ne jamais rebaser une branche déjà partagée ou mergée.
- Commits courts, message clair à l'impératif : « ajoute… », « corrige… ».
- Ne pas committer de données ni de secrets : `*.csv`, `*.log`, `.venv/`,
  fichiers de calibration (`invA.csv`, `xbar.csv`, `donnees_mag.csv`,
  `magNorthWest.csv`) restent locaux au bateau. Le `.gitignore` s'en charge.
- Avant d'ouvrir une PR : `python -m py_compile <fichiers modifiés>` et, si
  possible, exécuter le code concerné.
- `GeoFencing.py` et `drivers/ddboatclass.py` sont partagés : prévenir l'autre
  avant de les modifier, et garder les PR petites.
- Se relire mutuellement avant de merger : au moins un commentaire ou une
  approbation sur la PR.

## Dépannage express

```bash
git status                                  # où j'en suis
git log --oneline --graph --all --decorate  # vue de l'historique
git pull --rebase origin main               # intégrer le travail distant
git push                                    # publier
```

Si `git push` est refusé (non-fast-forward) : quelqu'un a poussé entre-temps.

```bash
git fetch origin
git rebase origin/main
git push
```

Si une branche locale est restée sur un ancien commit après un rebase de
`main`, la remettre à jour :

```bash
git switch ma-branche
git rebase main
```
