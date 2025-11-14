import numpy as np

def angle_to_impulsion(angle, min_angle=-90, max_angle=90, min_imp=500, max_imp=2500):
    """
    Convertit un ou plusieurs angles en impulsion(s) servo proportionnelle(s).
    Compatible avec des scalaires ou des tableaux NumPy.
    """
    angle = np.clip(angle, min_angle, max_angle)
    impulsion = min_imp + (angle - min_angle) * (max_imp - min_imp) / (max_angle - min_angle)
    return impulsion


def impulsion_to_angle(impulsion, min_angle=-90, max_angle=90, min_imp=500, max_imp=2500):
    """
    Convertit une ou plusieurs impulsions servo en angle(s) proportionnel(s).
    Compatible avec des scalaires ou des tableaux NumPy.
    """
    impulsion = np.clip(impulsion, min_imp, max_imp)
    angle = min_angle + (impulsion - min_imp) * (max_angle - min_angle) / (max_imp - min_imp)
    return angle

def RtRobot_cmd(servos_pos,impulsions,speed,delay,gripper="1200") :

    #speed 500 rapide .... 2000 lent
    #delay en millisecond : délai d'attente avant de rendre la main
    #Etat du gripper par defaut "ouvert"
    
    num_servo = 0
    
    cmd = ''

    for i in impulsions :
        cmd = cmd + "#" + servos_pos[num_servo] + "P" + str(int(i))
        num_servo += 1
    cmd = cmd + "#1P" + gripper + "T" + speed + "D" + delay
    
    return cmd
    
    
# # Exemple avec une seule valeur
# print(angle_to_impulsion(45))  # → 2250

# # Exemple avec plusieurs angles
# angles = np.array([-90, -45, 0, 45, 90])
# impulsions = angle_to_impulsion(angles)
# print(impulsions)  # → [   0.  750. 1500. 2250. 3000.]

# # Exemple avec plusieurs angles avec changement du max_imp
# angles = np.array([-90, -45, 0, 45, 90])
# impulsions = angle_to_impulsion(angles,max_imp=2500)
# print(impulsions)  # → [   0.  625. 1250. 1875. 2500.]

# # Exemple avec plusieurs angles avec changement de min_angle et max_angle
# angles = np.array([-180, -90, 0, 90, 180])
# impulsions = angle_to_impulsion(angles,min_angle=-180, max_angle=180)
# print(impulsions)  # → [   0.  625. 1250. 1875. 2500.]

# # Et l’inverse :
# angles_calc = impulsion_to_angle(impulsions)
# print(angles_calc)  # → [-90. -45.   0.  45.  90.]
