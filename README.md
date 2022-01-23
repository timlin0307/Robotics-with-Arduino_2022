# Robotics-with-Arduino_2022

---
## Introduction
- C'est un projet de cours Project Thématique à ECM (2A). L’objectif de notre projet transverse est de construire un robot pour pouvoir participer à la Coupe de France de Robotique (CFR) 2022.
- Pour ce faire, nous devons construire un robot totalement autonome qui devra réaliser le plus d’actions prédéfinies par le cahier des charges de la compétition afin de prendre le plus de points possible.

---
## Règlement de CFR Général
- Cette année, la CFR a pour thème : « Age of Robots ».
- Nous devons donc concevoir un robot et un bras manipulateur qui vont devoir réaliser des tâches en rapport avec ce thème afin de gagner des points. Lors du concours, les robots des différentes équipes se rencontrent lors de matchs opposants deux équipes sur un terrain de 2 mètres par 3 mètres :
  <p align="center">
    <img src="https://user-images.githubusercontent.com/54052564/150689646-fb178456-1316-4dc5-a849-505d54623704.png" />
  </p>
- Les robots de chaque équipe commencent dans leurs aires de départ respectives et ont alors 100 secondes pour réaliser différentes tâches afin de gagner un maximum de points.

---
## Stratégie
- L’équipe a jugé que de retourner chaque échantillon et de le placer au bon endroit serait trop compliqué techniquement et trop chronophage.
- Ainsi tel est notre stratégie : (Total de points de la stratégie choisie : 72 points)
  - **Gagner des points avec la statuette** : Total : 29 pts
    - Nous pensons qu’il serait intéressant de faire une statuette nous même que l’on placerait sur le pied d’estale (2 pts).
    - Puis, le robot enlèverait la statuette du pied d’estale (5pts) pour la mettre sur la vitrine (15pts) (vitrine est aussi faite par l’équipe: 2 pts).
    - En posant la statuette, la vitrine s’illumine (5 pts).
  - **On déplace les échantillons** : Total : 13 pts
    - En priorité, déplacer les échantillons rapportant le plus de points.
    - Ceux devant être déposés dans la galerie (3 pts chacun pour un total de 9).
    - Ceux devant être enlevés des distributeurs (1 pts chacun pour un total de 4).
  - **Les carrés de fouilles** : Total: 10 pts
    - La stratégie serait de prendre chaque résistance une par une et de retourner un seul carreau appartenant à notre équipe (10pts). Faire plus prendrait trop de temps, et retourner un carreau rouge nous ferait perdre l’intégralité des points gagnés dans cette épreuve.
  - **Revenir au campement à la fin** : Total : 20 pts

---
## Annuaires
```
Robotics with Arduino 2022 :.
├─ Libraries
│  ├─ RPLidarDriver
│  │  ├─ RPLidar.cpp
│  │  ├─ RPLidar.h
│  │  └─ ...
│  │
│  └─ SimpleTimer
│     ├─ SimpleTimer.cpp
│     ├─ SimpleTimer.h
│     └─ ...
│
├─ Robot_Trajet
│  └─ Robot_Trajet.ino
│
└─ README.md
```

---
## Schémas
- En cours de traitement

---
## Travail Réalisé
1. **Le Moteur avec Encodeur** : Nous comptons le nombre de tours du moteur en détectant les impulsions numériques émises par l'encodeur. Sur la base de ces détections, nous utilisons un controleur Proportional–Integral–Derivative (PID) pour rendre le nombre de tours du moteur plus précis, afin que nous puissions faire en sorte que le robot atteigne exactement n'importe quelle position que nous décidons.
2. **Lidar**
