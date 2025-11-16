# 📁 Projet 5dof-arm-camera-piloted

## Description Générale

Piloter avec une caméra un bras motorisé (5DOF + Gripper) afin de saisir un objet et le déplacer dans un pot

Il y a deux programmes : 

Programme 1) Déterminer la position d'un objet avec une caméra et des marqueurs Aruco (Aruco_detection.py)

Plan : 4 marqueurs ARUCO

Objet: 1 marqueur ARUCO

Exporter la position de l'objet détecté sur le plan dans dans un fichier JSON
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
├── aruco/                  # Répertoire contenant les marqueurs aruco à imprimer
│   ├── *.svg
├── calibration/            # Répertoire contenant les fichiers de calibration de la caméra
│   ├── *.npz
├── concept/                # Répertoire contenant les images du concept
│   ├── img....
│── images_bras/            # Dossier pour les images du bras
│   ├── img....
│
```

## Le bras avec 5DOF + Gripper

![image 3](/concept/5dofs_arm_3.jpg)

Image avec la position des axes des servos

![image 4](/concept/5dofs_arm_4.jpg)


## La carte de pilotage du robot (RTrobot 32 channels)

Carte de pilotage du robot pilotée via le port série ou usb du PC

![image 7](/images_bras/RTrobot_32.jpg)

![image 8](/images_bras/RTrobot_command.jpg)

## Définition géométrique du robot et des paramètres DH

Les éléments constitutifs du robot sont :

1. Base – rotation autour de l’axe vertical (yaw)
2. Épaule – élévation/abaissement du bras
3. Coude – pliure principale
4. Poignet (pitch) – inclinaison vers le haut/bas
5. Poignet (yaw) – rotation du poignet
6. Gripper – ouverture/fermeture de la pince

Les 5 articulations (joints) sont :

1. Base (J1) : Rotation autour de l'axe vertical (rotation azimutale)
2. Épaule (J2) : Rotation qui lève/baisse le bras
3. Coude (J3) : Rotation qui plie le bras
4. Poignet 1 (J4) : Rotation qui plie le poignet
5. Poignet 2 (J5) : Rotation finale de l'effecteur (yaw)

Gripper : Généralement considéré à part (ouverture/fermeture)

Les mesures : 

	'L1': 0.082,  # Hauteur de la base (du sol au centre de l'axe du bas de l'épaule)
	'L2': 0.099,  # Longueur bras supérieur (épaule-coude)
	'L3': 0.134,  # Longueur avant-bras (coude-poignet)
	'L4': 0.070,  # Longueur poignet (offset vertical)
	'L5': 0.075   # Longueur effecteur final (offset vertical)

![image 5](/concept/5dofs_arm_5.jpg)

Paramètres DH

 Format: [a, alpha, d, theta]
 
 Convention: 
 
             J2=0 -> bras horizontal vers l'avant
 
             J2>0 -> bras monte

             J2<0 -> bras descend

dh_params = [		

	[0           , np.pi/2, self.L['L1'], θ[0]],               		 J1: Base (rotation azimutale)

	[self.L['L2'], 0      , 0           , θ[1]+np.pi/2],       		 J2: Épaule (offset +90° pour horizontal à 0)

	[self.L['L3'], 0      , 0           , θ[2]-np.pi/2],       		 J3: Coude

	[0           , np.pi/2, 0           , θ[3]+np.pi/2],       		 J4: Poignet 1 (rotation dans le plan)

	[0           ,0       ,self.L['L4'] , θ[4]],               		 J5: Poignet 2 (rotation finale yaw)

	[0           ,0       ,self.L['L5'] , 0]                   		 Effecteur (grippeur)

]		


## Dépendances et Installation


### Installation des Bibliothèques

```bash
pip install numpy cv2 json os scipy matplotlib mpl_toolkits serial time

```

## Configuration et Exécution

### Sur un PC Standard (Webcam USB)

Ouvrir deux terminaux de commade :

Dans le premier :

1. python Aruco_detection.py

![image 6](/images_bras/find_obj_coord.jpg)

Dans le deuxième :

2. python robot_arm_inverse_k.py

![gif 1](/images_bras/move_bras.gif)

## Notes et Points d'Attention

Le réferentiel "monde" contenant le plan et l'objet ont X=0, Y=0, Z=0 au centre du plan délimité par les 4 marqueurs.

La position du pot est fixe , il faut rentrer ses coordonnées en mm dans le programme robot_arm_inverse_k.py à la ligne contenant :

        #Drop the parcel
        x=40
        y=109.6
        z=150

La valeur z = 150 est pour indiquer à quelle hauteur doit se positionner le bras avant de lacher l'objet.

Le positionnemt du bras sur le plan et les différentes mesures sont les points les plus délicats à mesurer car cela influe directement sur le bon calcul de position en Kinematic Inverse.