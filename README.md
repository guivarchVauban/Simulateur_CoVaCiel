# 🎥 Tutoriel ROS2 Humble — Intégration de Webots avec ROS2 (Ubuntu 22.04 VM)

Lien tuto vidéo associé: [https://youtu.be/Sje9Hwl2hgo]

Ce second tutoriel complète la série ROS2 Humble et introduit l’intégration du simulateur **Webots** avec **ROS2** dans un environnement **Ubuntu 22.04** installé sur **VirtualBox**.

L’objectif est de comprendre :
- la mise en place complète de l’environnement
- le fonctionnement de Webots
- la structure des fichiers de simulation
- le lien entre Webots et ROS2 via le bridge officiel

---

# 📚 Sommaire de la vidéo

## 1️⃣ Installation de l’environnement de travail

### VM Ubuntu 22.04
Installation d’une machine virtuelle Ubuntu Desktop 22.04.

🔗 Image ISO officielle :  
https://releases.ubuntu.com/jammy/ubuntu-22.04.5-desktop-amd64.iso

---

### Installation de ROS2 Humble

Installation de ROS2 Humble via les paquets Debian officiels.

🔗 Guide d’installation :  
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

---

### Installation de Webots

Installation du simulateur Webots via APT.

🔗 Procédure officielle :  
https://cyberbotics.com/doc/guide/installation-procedure#installing-the-debian-package-with-the-advanced-packaging-tool-apt

---

## 2️⃣ Fonctionnement de Webots

### Généralités

Présentation de Webots :
- simulateur robotique 3D
- environnement de test pour robots mobiles, manipulateurs, capteurs
- contrôleurs programmables
- intégration ROS2 via un bridge dédié

---

### Fichier Monde (`.wbt`)

Le fichier **World** définit :
- la scène complète
- les robots
- les objets
- la physique
- l’environnement

C’est le fichier principal chargé par Webots.

---

### Fichier Proto

Les fichiers **PROTO** permettent :
- de définir des robots ou objets réutilisables
- de paramétrer des modèles
- de créer des briques modulaires
- de simplifier les mondes complexes

Ils fonctionnent comme des “classes” de modèles 3D.

---

### Fichier Contrôleur (ROS / Webots)

Le contrôleur est le programme qui pilote le robot :
- contrôleur Webots natif
- ou contrôleur ROS2
- permet de lire les capteurs
- envoyer des commandes
- publier / souscrire à des topics ROS2

---

## 🔗 Bridge Webots ↔ ROS2

Le bridge officiel permet :
- de connecter Webots à ROS2
- de publier les capteurs du robot
- de commander les actionneurs
- d’intégrer la simulation dans un écosystème ROS2

🔗 Documentation officielle :  
https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Simulation-Webots.html

---
