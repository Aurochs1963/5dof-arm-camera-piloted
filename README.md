# 🤖 Projet de Bras Robotique 5-DOF Piloté par Caméra

## 🎯 Objectif du Projet

Ce projet a pour but de piloter un bras robotique à 5 degrés de liberté (5-DOF) et une pince (gripper) à l'aide d'une caméra. L'objectif est de détecter un objet marqué par un tag ArUco, de le saisir, puis de le déposer dans un emplacement prédéfini (un pot).

Le système se compose de deux applications principales qui communiquent via un fichier JSON :

1.  **Détection par Caméra (`Aruco_detection.py`)**:
    *   Utilise une caméra pour identifier un plan de travail défini par 4 marqueurs ArUco.
    *   Détecte un objet cible portant un autre marqueur ArUco.
    *   Calcule les coordonnées 3D (x, y, z) de l'objet par rapport au centre du plan.
    *   Exporte ces coordonnées en temps réel dans le fichier `positions.json`.

2.  **Contrôle du Bras (`robot_arm_inverse_k.py`)**:
    *   Lit les coordonnées de l'objet depuis `positions.json`.
    *   Utilise la **cinématique inverse** pour calculer les angles de rotation nécessaires pour chaque servo du bras.
    *   Génère et envoie les commandes de mouvement à la carte de contrôle du bras via une liaison série.
    *   Exécute une séquence complète : approcher, saisir, soulever, déplacer et déposer l'objet.

!Concept du projet

![image 1](/concept/5dofs_arm_1.jpg)
![image 2](/concept/5dofs_arm_2.jpg)

## 📂 Structure des Fichiers

```
/
├── README.md               # Ce fichier de documentation
├── Aruco_detection.py      # Script de détection de l'objet par caméra
├── robot_arm_inverse_k.py  # Script de pilotage du bras (cinématique inverse)
├── utils_servos.py         # Fonctions utilitaires pour le contrôle des servos
├── camera_calibration.npz  # Fichier de données de calibration de la caméra
├── positions.json          # Fichier d'échange de coordonnées (généré par Aruco_detection.py)
├── description.xlsx        # Paramètres DH et configuration du robot
├── aruco/                    # Marqueurs ArUco à imprimer
├── calibration/            # Scripts et images pour la calibration de la caméra
└── concept/                  # Images et schémas conceptuels du projet
```

## 🛠️ Matériel Requis

1.  **Bras Robotique** : Un bras 5-DOF avec une pince (gripper).
2.  **Carte de Contrôle** : Une carte de contrôle pour servos, comme la **RTrobot 32 channels**, connectée au PC via USB/Série.
3.  **Caméra** : Une webcam USB standard.
4.  **Marqueurs ArUco** :
    *   4 marqueurs `DICT_4X4_50` (IDs: 1, 2, 3, 4) pour délimiter le plan.
    *   1 marqueur `DICT_ARUCO_ORIGINAL` pour l'objet.

!Schéma des axes du bras

![image 3](/concept/5dofs_arm_3.jpg)

!Image avec la position des axes des servos

![image 4](/concept/5dofs_arm_4.jpg)

!Carte de pilotage du robot pilotée via le port série ou usb du PC

![image 7](/images_bras/RTrobot_32.jpg)

![image 8](/images_bras/RTrobot_command.jpg)


## ⚙️ Définition Géométrique et Paramètres DH

Le bras est modélisé avec 5 articulations (joints) et des segments de longueurs fixes. Ces paramètres sont essentiels pour les calculs de cinématique.

### Longueurs des Segments

Les longueurs sont définies dans `robot_arm_inverse_k.py` :
*   `L1`: 8.2 cm (Hauteur de la base)
*   `L2`: 9.9 cm (Bras supérieur : épaule -> coude)
*   `L3`: 13.4 cm (Avant-bras : coude -> poignet)
*   `L4`: 7.0 cm (Longueur du poignet)
*   `L5`: 7.5 cm (Longueur de l'effecteur final)

!Mesures du bras
![image 5](/concept/5dofs_arm_5.jpg)

### Paramètres Denavit-Hartenberg (DH)

Les paramètres DH sont utilisés pour calculer la position de l'effecteur (cinématique directe). Le format est `[a, alpha, d, theta]`.

```python
# Convention:
# J2=0 -> bras horizontal vers l'avant
# J2>0 -> bras monte
dh_params = [
    # J1: Base (rotation azimutale)
    [0, np.pi/2, self.L['L1'], θ],
    # J2: Épaule (offset +90° pour que 0° soit horizontal)
    [self.L['L2'], 0, 0, θ + np.pi/2],
    # J3: Coude
    [self.L['L3'], 0, 0, θ - np.pi/2],
    # J4: Poignet 1
    [0, np.pi/2, 0, θ + np.pi/2],
    # J5: Poignet 2 (rotation yaw)
    [0, 0, self.L['L4'], θ],
    # Effecteur (pince)
    [0, 0, self.L['L5'], 0]
]
```

## 🚀 Installation et Configuration

### 1. Dépendances Python

Installez les bibliothèques nécessaires avec `pip` :

```bash
pip install numpy opencv-python opencv-contrib-python pyserial matplotlib
```

### 2. Calibration de la Caméra

La précision de la détection dépend d'une bonne calibration.
*   Utilisez les scripts et images du dossier `/calibration` pour générer le fichier `camera_calibration.npz`.
*   Si ce fichier est absent, `Aruco_detection.py` utilisera des valeurs par défaut, mais la précision sera faible.

### 3. Configuration du Bras

Le script `robot_arm_inverse_k.py` contient des paramètres importants à vérifier :

*   **Port Série** : Assurez-vous que le port `COM` est correct.
    ```python
    ser = serial.Serial(port='COM3', baudrate=115200, timeout=0)
    ```

*   **Position de la Base** : Mesurez la position du centre de la base du robot par rapport au centre du plan de travail (défini par les marqueurs ArUco) et mettez à jour les coordonnées `(x, y, z)` en mètres.
    ```python
    # Position de la base du robot dans le repère "monde"
    base = (-0.155, 0, 0)
    robot = RobotArm5DOF(base_position=base)
    ```

*   **Position de Dépôt** : Configurez les coordonnées (en mm) où le bras doit déposer l'objet.
    ```python
    # Coordonnées pour déposer l'objet
    x = 40
    y = 109.6
    z = 150 # Hauteur de sécurité avant de lâcher
    ```

*   **Correction de l'Effecteur** : Des deltas `dx`, `dy`, `dz` (en mètres) peuvent être ajustés pour compenser un décalage entre le centre du marqueur de l'objet et le point de saisie réel de la pince.
    ```python
    dx = 0.02
    dy = 0.005
    dz = 0.03
    ```

## ▶️ Exécution du Projet

Pour lancer le système, ouvrez deux terminaux distincts.

### Terminal 1 : Détection de l'Objet

Lancez le script de détection. Une fenêtre s'ouvrira, affichant le flux de la caméra avec les marqueurs et les coordonnées calculées.

```bash
python Aruco_detection.py
```

!Fenêtre de détection
![image 6](/images_bras/find_obj_coord.jpg)

### Terminal 2 : Contrôle du Bras

Lancez le script de contrôle du bras. Il attendra vos instructions pour démarrer la séquence de pick-and-place.

```bash
python robot_arm_inverse_k.py
```

Le script vous demandera :
1.  `lecture valeur camera (o/n) ?` : Tapez `o` pour lire la dernière position depuis `positions.json`.
2.  `Envoi de la position au robot (o/n) ?` : Tapez `o` pour envoyer la commande de mouvement au bras.

Le bras exécutera alors la séquence pour saisir et déposer l'objet.

!GIF du mouvement du bras

![gif 1](/images_bras/move_bras.gif)


## ▶️ Version multi-objets

Ces versions permettent de détecter la position de plusieurs objet y compris le pot.

Dans cet exemple, les deux objet ont les tags 90 et 323 et le pot le tag 531

Pour lancer le système, ouvrez deux terminaux distincts.

### Terminal 1 : Détection des objets

Lancez le script de détection. Une fenêtre s'ouvrira, affichant le flux de la caméra avec les marqueurs et les coordonnées calculées.

```bash
python Aruco_multi_detection.py
```

### Terminal 2 : Contrôle du Bras

Lancez le script de contrôle du bras. Il attendra vos instructions pour démarrer la séquence de pick-and-place.

```bash
python robot_arm_inverse_km.py
```

Le script vous demandera :

1.  `lecture valeur camera (o/n) ?` : Tapez `o` pour lire la dernière position depuis `positions.json`.

Affichage de la position des différents objets dont le pot (tag 531)

--- Objets trouvés dans le fichier ---
ID: 323 -> Données: {'x': 115.3, 'y': -81.6, 'z': 1.8, 'xmoy': 119.6, 'ymoy': -80.1, 'zmoy': 13.7}
ID: 531 -> Données: {'x': 100.9, 'y': 77.8, 'z': 18.6, 'xmoy': 100.9, 'ymoy': 77.3, 'zmoy': 19.1}
ID: 90 -> Données: {'x': 47.4, 'y': -115.5, 'z': 61.8, 'xmoy': 49.7, 'ymoy': -115.0, 'zmoy': 66.1}
Position du pot (tag #531): xpot= 100.9 ypot= 77.8 zpot= 18.6

Entrez l'ID de l'objet que vous souhaitez choisir (ex: 90) :

il faudra sélectionner l'ID d'un objet (sauf celui du pot ...) pour indiquer l'objet à prendre

Le bras exécutera alors la séquence pour saisir et déposer l'objet dans le pot.


## ⚠️ Notes Importantes

*   **Précision des Mesures** : La performance du système dépend fortement de la précision des mesures physiques : longueurs des segments du bras, position de sa base, et calibration de la caméra. Toute erreur se répercutera sur la précision du positionnement.
*   **Référentiel Monde** : Le point d'origine du repère monde (0,0,0) est le **centre du plan** délimité par les 4 marqueurs ArUco. Toutes les coordonnées sont calculées par rapport à ce point.
*   **Sécurité** : Soyez prudent lors des premiers essais. Les mouvements du bras peuvent être inattendus. Gardez une main sur l'alimentation pour pouvoir l'arrêter en cas d'urgence.