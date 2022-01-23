# Robotics-with-Arduino_2022

---
## Introduction
- C'est un projet de cours Project Thématique à ECM (2A). Le projet a pour objectif de se familiariser avec les problématiques rencontrées en robotique, il s’organisera en synergie avec le groupe d’élèves de 2A participant à la coupe de France de robotique.
- L’objectif de ce projet est représentation de l’École au plus grand événement de robotique national : La Coupe de France de Robotique (CFR) 2021.

---
## Règlement de CFR Général
- Cette année (comme l’an dernier) , la CFR a pour thème : « Sail the World »; et se concentre sur le domaine maritime.
- Nous devons donc concevoir un robot et un phare qui vont devoir réaliser des tâches en rapport avec ce thème afin de gagner des points. Lors du concours, les robots des différentes équipes se rencontrent lors de matchs opposants deux équipes sur un terrain de 2 mètres par 3 mètres :
  <p align="center">
    <img src="https://user-images.githubusercontent.com/54052564/150683843-1a4257ba-3c8e-430c-8533-80842a31d256.png" />
  </p>
- Les robots de chaque équipe commencent dans leurs aires de départ respectives et ont alors 100 secondes pour réaliser différentes tâches afin de gagner un maximum de points.

---
## Stratégie
- En cours de traitement

---
## Annuaires
```
Robotics with Arduino 2022 :.
├─ Libraries
│  ├─ BaseRoulante
│  │  ├─ BaseRoulante.cpp
│  │  ├─ BaseRoulante.h
│  │  └─ ...
│  │
│  ├─ DCMotor
│  │  ├─ DCMotor.cpp
│  │  ├─ DCMotor.h
│  │  └─ ...
│  │
│  └─ PID_v1
│     ├─ PID_v1.cpp
│     ├─ PID_v1.h
│     └─ ...
│
├─ Phare
│  └─ Phare.ino
│
├─ SEN-Color
│  └─ SEN-Color.ino
│
├─ Trajet
│  └─ Trajet.ino
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
