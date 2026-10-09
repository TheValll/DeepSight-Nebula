# Benchmark du pipeline

Mesure de base faite le 30 septembre 2026 avec un bon éclairage, RViz et MoveIt en mode planification.
Le test utilise 5 secondes de chauffe puis 30 secondes de mesure.

| Étape | Médiane | P95 |
|:---|---:|---:|
| Période caméra | 67.58 ms | 68.91 ms |
| Lecture et décodage caméra | 46.10 ms | 51.23 ms |
| Rectification stéréo | 2.30 ms | 2.86 ms |
| Calcul de profondeur | 13.14 ms | 18.01 ms |
| Nuage de points | 10.83 ms | 12.60 ms |
| Traitement stéréo complet | 20.23 ms | 32.45 ms |
| Attente avant YOLO | 17.50 ms | 22.58 ms |
| Inférence YOLO | 37.78 ms | 43.58 ms |
| Traitement YOLO complet | 47.29 ms | 57.67 ms |
| Capture jusqu'à la détection | 65.64 ms | 77.39 ms |
| Calcul du goal 3D | 0.79 ms | 1.04 ms |
| Capture jusqu'au goal 3D | 66.78 ms | 78.33 ms |
| Premier filtre de position | 601.62 ms | 756.12 ms |
| Filtre de stabilité du replanning | 389.82 ms | 456.70 ms |
| Planification MoveIt | 15.05 ms | 15.90 ms |
| Requête MoveIt complète | 42.10 ms | 2865.38 ms |

La caméra demande bien 30 FPS, mais le pipeline tourne à environ 15 FPS. Les principaux coûts sont la lecture caméra
et YOLO. MoveIt est rapide et n'est pas le problème principal.

426 goals 3D valides ont été produits sur 428 images YOLO, soit 99.5 %. MoveIt a réussi 9 planifications sur 10.
Le P95 élevé de la requête MoveIt vient d'un seul échec long. Certaines étapes travaillent en parallèle : il ne faut
donc pas additionner toutes les lignes.

Le premier filtre publie immédiatement, mais utilise jusqu'à 0.6 seconde d'historique. Il peut donc donner une position
en retard quand la balle bouge. Le filtre de replanning attend réellement 6 positions stables, soit environ 0.39 seconde.

YOLOE-26m tourne bien sur la RTX 5080 avec CUDA. Une mesure instantanée pendant le test donnait 91 % de GPU et 910 MiB
de mémoire pour le processus Python.

## Après les premières optimisations

Mesure faite le 1er octobre 2026, sans changer le modèle YOLO.

| Étape | Avant | Après | Évolution |
|:---|---:|---:|---:|
| Période caméra | 67.58 ms | 67.72 ms | stable |
| Traitement stéréo | 20.23 ms | 15.82 ms | -21.8 % |
| Visualisations stéréo | 1.38 ms | 0.00 ms | -100 % |
| Dessin YOLO et goal 3D | 10.68 ms | 0.36 ms | -96.6 % |
| Traitement YOLO complet | 47.29 ms | 41.06 ms | -13.2 % |
| Capture jusqu'à la détection | 65.64 ms | 58.04 ms | -11.6 % |
| Capture jusqu'au goal 3D | 66.78 ms | 59.20 ms | -11.4 % |
| Historique du premier filtre | 601.62 ms | 269.29 ms | -55.2 % |
| Attente du filtre stable | 389.82 ms | 237.05 ms | -39.2 % |
| Goal jusqu'à la requête MoveIt | 58.70 ms | 30.79 ms | -47.5 % |
| Requête MoveIt complète | 42.10 ms | 36.39 ms | -13.6 % |

Cette première mesure restait limitée à 14.8 FPS. Le test suivant a permis de trouver que le buffer unique imposé à
OpenCV divisait la cadence par deux.

400 goals 3D valides ont été produits sur 443 images YOLO, soit 90.3 %. Le modèle n'a pas changé. MoveIt a réussi les
8 planifications demandées et aucun échec long n'a été observé. La mesure GPU instantanée donnait 99 % et 910 MiB pour
le processus Python.

## Après la correction du buffer caméra

La caméra est maintenant branchée sur le nouveau port USB et OpenCV utilise 2 buffers au lieu d'un seul.

| Étape | Baseline | Résultat final | Évolution |
|:---|---:|---:|---:|
| FPS caméra | 14.8 | 31.0 | +109 % |
| Période caméra | 67.58 ms | 32.27 ms | -52.2 % |
| Traitement stéréo | 20.23 ms | 15.65 ms | -22.6 % |
| Traitement YOLO complet | 47.29 ms | 36.70 ms | -22.4 % |
| Capture jusqu'à la détection | 65.64 ms | 60.82 ms | -7.3 % |
| Capture jusqu'au goal 3D | 66.78 ms | 61.96 ms | -7.2 % |
| Historique du premier filtre | 601.62 ms | 150.17 ms | -75.0 % |
| Attente du filtre stable | 389.82 ms | 142.42 ms | -63.5 % |
| Goal jusqu'à la requête MoveIt | 58.70 ms | 17.88 ms | -69.5 % |

Le pipeline a reçu 901 images caméra et traité 791 images YOLO en 30 secondes. Il produit donc environ 30 FPS côté
caméra et 26 FPS côté YOLO. 789 goals 3D valides ont été produits sur 791 images YOLO, soit 99.7 %.

MoveIt a réussi 7 planifications sur 12 pendant cette passe. Plusieurs goals étaient hors de la zone atteignable du
bras, donc son temps de planification ne doit pas être comparé à la baseline pour ce test.

Résultats bruts :

- `baseline_lit_2026-09-30.csv`
- `baseline_lit_2026-09-30.json`
- `after_core_optimizations_2026-10-01.csv`
- `after_core_optimizations_2026-10-01.json`
- `after_usb_buffer_fix_2026-10-01.csv`
- `after_usb_buffer_fix_2026-10-01.json`

Relancer la même mesure après chaque optimisation :

```bash
cd /home/valentin/DeepSight-Nebula/app
source ../.venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run stereo_camera_calibration pipeline_benchmark_node --ros-args \
  -p warmup_sec:=5.0 \
  -p duration_sec:=30.0 \
  -p label:=after_optimization \
  -p output_dir:=../benchmarks/pipeline
```
