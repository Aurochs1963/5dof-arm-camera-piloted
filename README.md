# 📁 Projet 5dof-arm-camera-piloted

## Description Générale

Piloter avec une caméra un bras motorisé (5DOF + Gripper) afin de saisir un objet et le déplacer dans un pot

Il y a deux programmes : 

Programme 1) Déterminer la position d'un objet avec une caméra et des marqueurs Aruco (Aruco_detection.py)

Plan : 4 marqueurs ARUCO

Objet: 1 marqueur ARUCO

Exporter la position de l'objet détecté sur le p^lan dans dans un fichier JSON
 data={x,y,z,xmoy,ymoy,zmoy}

![image 1](/concept/5dofs_arm_1.jpg)

Programme 2) Récupérer la position de l'objet, le prendre et le déposer dans un pot (robot_arm_inverse_k.py)

- Lire la position de  l'objet (json)
- Placer le bras au dessus de l'objet
- Prendre l'objet
- Relever le bras
- Déplacer le bras au dessus du pot
- Lâcher l'objet dans le pot

![image 2](/concept/5dofs_arm_2.jpg)

--> Transformer les coordonnées x,y,z de l'objet en angles de rotation des servos du bras (Inverse Kinematics).


## Structure des Fichiers

```
/project-root
│
├── README.md               # Ce fichier
├── Aruco_detection.py      # Application de détection de l'objet via la caméra
├── robot_arm_inverse_k.py  # Application de pilotage du bras
├── utils_servos.py         # Utilitaires de pilotage des servos
├── camera_calibration.npz  # Données de calibration de la caméra
├── positions.json          # Données de position de l'objet
├── description.xlsx        # Données de paramétrage du robot pour la matrice (IK/DH parameters) utilisée pour calculer la position du bras 
├── concept/                # Répertoire contenant le concept
│   ├── img....
│── images_bras/            # Dossier pour les images du bras
│   ├── img....
│
```

## Structure du robot

![image 1](/concept/5dofs_arm_3.jpg)

## Dépendances et Installation


### Installation des Bibliothèques

```bash
pip install numpy cv2 json os scipy matplotlib mpl_toolkits serial time
```

## Configuration et Exécution

### Sur un PC Standard (Webcam USB)


## Notes et Points d'Attention




