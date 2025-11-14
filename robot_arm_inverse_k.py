import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import utils_servos

import serial
import time

import json

class RobotArm5DOF:
    """
    Bras robotique à 5 degrés de liberté
    J1: Base (rotation azimutale)
    J2: Épaule (lève/baisse le bras)
    J3: Coude (plie le bras)
    J4: Poignet 1 (plie le poignet)
    J5: Poignet 2 (rotation finale yaw)
    """
    
    def __init__(self, link_lengths=None, base_position=None):
        """
        link_lengths: dict contenant les longueurs des segments
        base_position: tuple (x, y, z) pour la position de la base dans le repère monde
                       Par défaut: (0, 0, 0)
        """
        if link_lengths is None:
            self.L = {
                'L1': 0.082,   # Hauteur de la base
                'L2': 0.099,    # Longueur bras supérieur (épaule-coude)
                'L3': 0.134,  # Longueur avant-bras (coude-poignet)
                'L4': 0.070,  # Longueur poignet (offset vertical)
                'L5': 0.075   # Longueur effecteur final (offset vertical)
            }
            print("Dimensions du bras :",self.L)
        else:
            self.L = link_lengths

        # Position de la base dans le repère monde
        if base_position is None:
            self.base_position = np.array([0.0, 0.0, 0.0])
        else:
            self.base_position = np.array(base_position)
            
        # Limites des joints (en degrés)
        self.joint_limits = {
            'J1': (-90, 90),
            'J2': (-90, 90),
            'J3': (-90, 90),
            'J4': (-90, 90),
            'J5': (-180, 180)
        }

        # # Limites des joints (en degrés)
        # self.joint_limits = {
            # 'J1': (-180, 180),
            # 'J2': (-90, 90),
            # 'J3': (-135, 135),
            # 'J4': (-90, 90),
            # 'J5': (-180, 180)
        # }
    
    def dh_matrix(self, a, alpha, d, theta):
        """
        Calcule la matrice de transformation DH (Denavit-Hartenberg)
        a: longueur du lien
        alpha: torsion du lien
        d: offset le long de l'axe précédent
        theta: angle de rotation autour de l'axe précédent
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self, angles, return_all_frames=False):
        """
        Cinématique directe: calcule la position et orientation de l'effecteur
        
        angles: liste [θ1, θ2, θ3, θ4, θ5] en degrés
        return_all_frames: si True, retourne aussi toutes les positions intermédiaires
        
        Retourne:
        - position: (x, y, z) dans le repère monde
        - orientation: (roll, pitch, yaw) en radians
        - frames: liste des matrices de transformation (si return_all_frames=True)
        """
        # Conversion en radians
        θ = [np.radians(a) for a in angles]
        
        # Paramètres DH modifiés pour ce robot
        # Format: [a, alpha, d, theta]
        # Convention: J2=0 -> bras horizontal vers l'avant
        #             J2>0 -> bras monte
        #             J2<0 -> bras descend
        # dh_params = [
            # [0, np.pi/2, self.L['L1'], θ[0]],                    # J1: Base (rotation azimutale)
            # [self.L['L2'], 0, 0, θ[1] + np.pi/2],                # J2: Épaule (offset +90° pour horizontal à 0)
            # [self.L['L3'], 0, 0, θ[2]],                          # J3: Coude
            # [self.L['L4'], 0, 0, θ[3]],                          # J4: Poignet 1 (rotation dans le plan)
            # [0,self.L['L5'],0,θ[4]]                              # J5: Poignet 2 (rotation finale yaw)
        # ]
        dh_params = [
            [0           , np.pi/2, self.L['L1'], θ[0]],               # J1: Base (rotation azimutale)
            [self.L['L2'], 0      , 0           , θ[1]+np.pi/2],       # J2: Épaule (offset +90° pour horizontal à 0)
            [self.L['L3'], 0      , 0           , θ[2]-np.pi/2],       # J3: Coude
            [0           , np.pi/2, 0           , θ[3]+np.pi/2],       # J4: Poignet 1 (rotation dans le plan)
            [0           ,0       ,self.L['L4'] , θ[4]],               # J5: Poignet 2 (rotation finale yaw)
            [0           ,0       ,self.L['L5'] , 0]                   # Effecteur (grippeur)
        ]

        # Transformation de base (position de la base dans le repère monde)
        T_base = np.eye(4)
        T_base[:3, 3] = self.base_position
        
        # Calcul des matrices de transformation
        T = T_base.copy()
        frames = [T.copy()]
        
        for params in dh_params:
            T_i = self.dh_matrix(*params)
            T = T @ T_i
            frames.append(T.copy())
        
        # Extraction de la position (déjà dans le repère monde)
        position = T[:3, 3]
        
        # Extraction de l'orientation (angles d'Euler ZYX)
        R = T[:3, :3]
        
        # Calcul roll, pitch, yaw
        if R[2, 0] < 1:
            if R[2, 0] > -1:
                pitch = np.arcsin(-R[2, 0])
                roll = np.arctan2(R[2, 1], R[2, 2])
                yaw = np.arctan2(R[1, 0], R[0, 0])
            else:
                pitch = np.pi / 2
                roll = -np.arctan2(-R[1, 2], R[1, 1])
                yaw = 0
        else:
            pitch = -np.pi / 2
            roll = np.arctan2(-R[1, 2], R[1, 1])
            yaw = 0
        
        orientation = (roll, pitch, yaw)
        
        if return_all_frames:
            return position, orientation, frames
        else:
            return position, orientation
    
    def inverse_kinematics(self, target_pos, target_orient=None, initial_guess=None):
        """
        Cinématique inverse: calcule les angles des joints pour atteindre une position
        
        target_pos: (x, y, z) position cible dans le repère monde
        target_orient: (roll, pitch, yaw) orientation cible (optionnel)
        initial_guess: angles initiaux pour l'optimisation [θ1, θ2, θ3, θ4, θ5]
         
        Retourne: [θ1, θ2, θ3, θ4, θ5] en degrés ou None si pas de solution
        """
        if initial_guess is None:
            initial_guess = [0, 0, 0, 0, 0]
        
        def cost_function(angles):
            """Fonction de coût à minimiser"""
            pos, orient = self.forward_kinematics(angles)
            
            # Erreur de position
            pos_error = np.linalg.norm(np.array(pos) - np.array(target_pos))
            
            # Erreur d'orientation (si spécifiée)
            if target_orient is not None:
                orient_error = sum((o1 - o2)**2 for o1, o2 in zip(orient, target_orient))
                return pos_error + 0.1 * orient_error
            
            return pos_error
        
        def constraint_function(angles):
            """Contraintes sur les limites des joints"""
            penalties = []
            for i, (angle, (j_min, j_max)) in enumerate(zip(angles, self.joint_limits.values())):
                if angle < j_min:
                    penalties.append((j_min - angle)**2)
                elif angle > j_max:
                    penalties.append((angle - j_max)**2)
            return -sum(penalties) if penalties else 0
        
        # Limites pour l'optimisation
        bounds = list(self.joint_limits.values())
        
        # Optimisation
        result = minimize(
            cost_function,
            initial_guess,
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 1000, 'ftol': 1e-6}
        )
        
        if result.success and result.fun < 0.01:  # Erreur < 1 cm
            return result.x.tolist()
        else:
            return None
    
    def plot_robot(self, angles, ax=None):
        """
        Visualise le bras robotique dans l'espace 3D
        
        angles: [θ1, θ2, θ3, θ4, θ5] en degrés
        ax: axes matplotlib (créé automatiquement si None)
        """
        if ax is None:
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')
        
        # Obtenir toutes les positions intermédiaires
        _, _, frames = self.forward_kinematics(angles, return_all_frames=True)
        
        # Extraire les positions de chaque joint
        positions = [frame[:3, 3] for frame in frames]
        x_coords = [p[0] for p in positions]
        y_coords = [p[1] for p in positions]
        z_coords = [p[2] for p in positions]
        
        # Tracer les segments du bras
        ax.plot(x_coords, y_coords, z_coords, 'o-', linewidth=3, markersize=8, 
                color='royalblue', label='Bras robotique')
        
        # Marquer la base et l'effecteur
        ax.scatter(*positions[0], color='green', s=200, marker='s', label='Base')
        ax.scatter(*positions[-1], color='red', s=200, marker='^', label='Effecteur')
        
        # Configuration des axes
        max_reach = sum(self.L.values())
        ax.set_xlim([-max_reach, max_reach])
        ax.set_ylim([-max_reach, max_reach])
        ax.set_zlim([0, max_reach])
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Configuration: J1={angles[0]:.1f}°, J2={angles[1]:.1f}°, '
                     f'J3={angles[2]:.1f}°, J4={angles[3]:.1f}°, J5={angles[4]:.1f}°')
        ax.legend()
        ax.grid(True)
        
        return ax

#Serial communication with robot
# Ouvre le port série
ser = serial.Serial(
    port='COM3',       # Remplace par ton port série (ex: '/dev/ttyUSB0' sous Linux)
    baudrate=115200,   # Vitesse de communication (à adapter à ton matériel)
    timeout=0          # Temps d’attente pour lecture (en secondes)
)

time.sleep(2)  # Petite pause pour laisser le port s'initialiser

def send_to_robot(commande):
    # Envoi de la commande (convertie en bytes ASCII)
    ser.write(commande.encode('ascii'))

    print("Commande envoyée :", commande)

    # Attente de la réponse
    reponse = ""
    start_time = time.time()
    timeout_max = 5  # secondes

    print("En attente de réponse...")

    while (time.time() - start_time) < timeout_max:
        if ser.in_waiting > 0:
            reponse = ser.readline().decode('ascii', errors='ignore').strip()
            if reponse:
                print("Réponse reçue :", reponse)
                break
    else:
        print("⚠️ Aucune réponse reçue après", timeout_max, "secondes")
        
        
def move_bras (target,gripper="1000",speed="1000",delay="500",sendcommand=None):

        print(f"\nPosition cible (dans le repère monde): x={target[0]}, y={target[1]}, z={target[2]}")
       
        solution = robot.inverse_kinematics(target)

        if solution:
            print(f"\nSolution trouvée:")
            print(f"Angles         : {[f'{a:.2f}°' for a in solution]}")

            angles_correct = solution * sens_to_serv
            print(f"Angles corrigés: {[f'{a:.2f}°' for a in angles_correct]}")
            
            impulsions[0] = utils_servos.angle_to_impulsion(angles_correct[0], min_angle=-135, max_angle=135)
            impulsions[1] = utils_servos.angle_to_impulsion(angles_correct[1], min_angle=-90, max_angle=90)
            impulsions[2] = utils_servos.angle_to_impulsion(angles_correct[2], min_angle=-90, max_angle=90)
            impulsions[3] = utils_servos.angle_to_impulsion(angles_correct[3], min_angle=-90, max_angle=90)
            impulsions[4] = utils_servos.angle_to_impulsion(angles_correct[4], min_angle=-90, max_angle=90)
            print(f"Impulsions     : {[f'{a:.0f}' for a in impulsions]}")  # → [   0.  750. 1500. 2250. 3000.]
            RtRobot_cmd = utils_servos.RtRobot_cmd(servos_pos,impulsions,speed,delay,gripper)
            print("Commande RtRobot :",RtRobot_cmd)

            # Vérification
            pos_check, _ = robot.forward_kinematics(solution)
            error = np.linalg.norm(np.array(pos_check) - np.array(target))
            print(f"\nVérification - Position atteinte: x={pos_check[0]:.4f}, y={pos_check[1]:.4f}, z={pos_check[2]:.4f}")
            print(f"Erreur: {error*1000:.2f} mm")
            # Visualisation
            #robot.plot_robot(solution)
        else:
            print("Aucune solution trouvée (position hors d'atteinte)")
     
        print("="*70)

        #plt.tight_layout()
        #plt.show()
        
        if solution:
            commande = RtRobot_cmd + "\r\n"
            if sendcommand is None:
                sendcommand = input("Envoi de la position au robot (o/n) ? :")
                if sendcommand =='o':
                    send_to_robot(commande)
            else:
                send_to_robot(commande)
        
# =============================================================================
# EXEMPLES D'UTILISATION
# =============================================================================

if __name__ == "__main__":

    # Créer une instance du bras robotique avec position de base personnalisée

    base = (-0.155, 0, 0)
    robot = RobotArm5DOF(base_position=base)
    
    print("="*70)
    print(f"Robot avec centre de la base à x={base[0]:.2f}, y={base[1]:.2f}, z={base[2]:.2f}")
    print("="*70)


    # Description des servos et sens de rotation, vitesse et délai
    servos_pos = ["6","5","4","3","2","1"]
    servos_sens = [1,1,1,-1,1]
    sens_to_serv = np.array(servos_sens)
    speed="800"
    delay="200"

    impulsions = ['','','','','']

    json_filename = "positions.json"
    
    #Deltas de correction de la position de l'effecteur (en mètres)
    dx = 0.02
    dy = 0.005
    dz = 0.03
        
    # x,y,z caméra = positions en mm de la cible dans le référentiel monde
    #                il faudrat les convertir en mètres avant de les envoyer au bras 
    # target = ((x/1000)+dx,(y/1000)+dy,(z/1000)+dz)
    # move_bras(target,"1000")   

    # Cinématique inverse  (dans le repère monde)
    print("="*70)
    print("Cinématique inverse")
    print("="*70)
   
    #Initialisation de la position du bras
    target = (0.04,0.0,0.12)
    move_bras(target,"1000",speed,delay,"n")   

    while True:
        # # Position cible
        lect = input("lecture valeur camera (o/n) ? :")
        if lect == "o":
            try:
                 with open(json_filename,"r") as f:
                     data = json.load(f)
            except json.JSONDecodeError as e:
                 print("Invalid JSON file:", e)
                 data = {"frame":0,"x":40.0,"y":0.0,"z":120.0,"xmoy":40.0,"ymoy":0.0,"zmoy":120.0}
                 print("valeurs par défaut:",data)
            frame = int(data['frame'])
            x = float(data['x'])
            y = float(data['y'])
            z = float(data['z'])
            xmoy = float(data['xmoy'])
            ymoy = float(data['ymoy'])
            zmoy = float(data['zmoy'])
            print("frame:",frame,"x:",x,"y:",y,"z:",z,"xmoy:",xmoy,"ymoy",ymoy,"zmoy",zmoy)

        #choix = input("valeur cam réelle, cam moyenne ou saisie (r/m/s) ? :")
        choix = "r"
     
        if choix == "m":
             x = xmoy
             y = ymoy
             z = zmoy
        if choix == "s":
             x,y,z = map(float,input("Input values x y z: ").split())
        
        #Approche
        target = ((x/1000)+dx,(y/1000)+dy,(z/1000)+dz)
        move_bras(target,"1000",speed,delay)   

        #Descente
        target = ((x/1000)+dx,(y/1000)+dy,(z/1000)-dz)
        move_bras(target,"1000",speed,delay,"n")   

        #Serrage
        target = ((x/1000)+dx,(y/1000)+dy,(z/1000)-dz)
        move_bras(target,"1600",speed,delay,"n")   

        #Remontée
        target = ((x/1000)+dx,(y/1000)+dy,(z/1000)+0.04)
        move_bras(target,"1600",speed,delay,"n")   

        #Drop the parcel
        x=40
        y=109.6
        z=150
        
        target = ((x/1000)+dx,y/1000,(z/1000))
        move_bras(target,"1600",speed,delay,"n")   

        target = ((x/1000)+dx,y/1000,(z/1000)-0.05)
        move_bras(target,"1000",speed,delay,"n")   
