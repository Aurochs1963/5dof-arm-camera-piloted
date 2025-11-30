import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
import numpy as np
import json

# --- PARAMÈTRES ---
aruco_dict_board = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_dict_object = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

plan_marker_length = 30.0   # mm
plan_ids = [1, 2, 3, 4]     # IDs des coins du plan
object_marker_length = 18.0 # mm
object_height = 38.0        # mm
REFERENCE_Z = "dessus"      # "dessus", "centre" ou "sol"
json_filename = "multi_positions.json"

SAVE_INTERVAL = 30          # Sauvegarde le JSON toutes les X frames

# Dictionnaire pour stocker l'historique de position de chaque ID (pour le lissage)
# Format: { id_int: [(x,y,z), (x,y,z)...] }
objets_history = {}
max_history_len = 10

# Charger la calibration si dispo
try:
    calib = np.load("camera_calibration.npz")
    camera_matrix = calib["camera_matrix"]
    dist_coeffs = calib["dist_coeffs"]
    print("Calibration chargée avec succès.")
except:
    print("Aucune calibration trouvée, utilisation de valeurs par défaut.")
    camera_matrix = np.array([[800, 0, 320],
                             [0, 800, 240],
                             [0, 0, 1]], dtype=float)
    dist_coeffs = np.zeros((5, 1))
    
# --- Initialisation caméra ---
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

frame_count = 0

print("=== Suivi Multi-ArUco - Détection automatique du plan ===")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Détection des marqueurs
    corners4x4, ids4x4, _ = cv2.aruco.detectMarkers(gray, aruco_dict_board)
    cornersObj, idsObj, _ = cv2.aruco.detectMarkers(gray, aruco_dict_object)

    # Dessin des marqueurs
    cv2.aruco.drawDetectedMarkers(frame, corners4x4, ids4x4)
    cv2.aruco.drawDetectedMarkers(frame, cornersObj, idsObj, (0, 255, 0))

    # --- 1. Détermination automatique du plan (Référentiel) ---
    plan_detecte = False
    R_inv = None
    centre_plan = None
    Z_axis = None

    if ids4x4 is not None and len(ids4x4) >= 4:
        corners_3d = {}
        # Estimation pose des coins du plan
        for i, id_ in enumerate(ids4x4.flatten()):
            if id_ in plan_ids:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [corners4x4[i]], plan_marker_length, camera_matrix, dist_coeffs
                )
                corners_3d[int(id_)] = tvecs[0][0]
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                                  rvecs[0], tvecs[0], plan_marker_length / 2)

        # Si tous les coins du plan sont vus
        if all(id_ in corners_3d for id_ in plan_ids):
            plan_detecte = True
            P1, P2, P3, P4 = [np.array(corners_3d[i]) for i in plan_ids]
            
            largeur = np.linalg.norm(P2 - P1)
            hauteur = np.linalg.norm(P1 - P4)
            centre_plan = (P1 + P2 + P3 + P4) / 4.0

            cv2.putText(frame, f"Plan: {largeur:.1f}x{hauteur:.1f} mm",
                        (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
            
            # Construction du système de coordonnées du plan
            X_axis = (P4 - P1)
            X_axis /= np.linalg.norm(X_axis)

            Y_axis = (P2 - P1)
            Y_axis /= np.linalg.norm(Y_axis)

            # Normale au plan (Z)
            Z_axis = np.cross(X_axis, Y_axis)
            Z_axis /= np.linalg.norm(Z_axis)

            # Matrice de rotation (Plan -> Caméra) puis inverse (Caméra -> Plan)
            R_board = np.column_stack((X_axis, Y_axis, Z_axis))
            R_inv = R_board.T 


    # --- 2. Traitement des Objets Multiples ---
    current_frame_data = {} # Dictionnaire pour le JSON de cette frame

    if idsObj is not None and len(idsObj) > 0:
        # On estime la pose de TOUS les marqueurs objets en une fois
        rvecs_obj, tvecs_obj, _ = cv2.aruco.estimatePoseSingleMarkers(
            cornersObj, object_marker_length, camera_matrix, dist_coeffs
        )

        # On boucle sur chaque objet détecté
        for i in range(len(idsObj)):
            current_id = int(idsObj[i][0])
            
            # Récupération vecteurs de cet objet spécifique
            rvec_i = rvecs_obj[i]
            tvec_i = tvecs_obj[i]

            # Dessin repère objet
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec_i, tvec_i, object_marker_length)

            if plan_detecte:
                # Position de l'objet dans le repère caméra
                pos_cam_obj = tvec_i.reshape(3, 1)
                pos_plan_origin = centre_plan.reshape(3, 1)

                # Projection dans le repère du plan
                pos_obj_plan = np.dot(R_inv, (pos_cam_obj - pos_plan_origin))

                # Hauteur Z géométrique (projetée sur la normale)
                z_geo = np.dot(Z_axis, (pos_cam_obj.flatten() - centre_plan))

                x, y = pos_obj_plan[0, 0], pos_obj_plan[1, 0]
                z = z_geo

                # Ajustement Z selon référence
                z_offset = 0.0
                if REFERENCE_Z == "centre":
                    z -= object_height / 2
                elif REFERENCE_Z == "sol":
                    z -= object_height
                z -= z_offset

                # --- Gestion de l'historique (Moyenne Mobile) par ID ---
                if current_id not in objets_history:
                    objets_history[current_id] = []
                
                objets_history[current_id].append((x, y, z))
                if len(objets_history[current_id]) > max_history_len:
                    objets_history[current_id].pop(0)

                # Calcul des moyennes
                xmoy = np.mean([p[0] for p in objets_history[current_id]])
                ymoy = np.mean([p[1] for p in objets_history[current_id]])
                zmoy = np.mean([p[2] for p in objets_history[current_id]])

                # Stockage des données pour le JSON
                current_frame_data[current_id] = {
                    "x": round(x, 1),
                    "y": round(y, 1),
                    "z": round(z, 1),
                    "xmoy": round(xmoy, 1),
                    "ymoy": round(ymoy, 1),
                    "zmoy": round(zmoy, 1)
                }

                # Affichage texte à côté de l'objet (coin supérieur gauche du marqueur)
                c_x, c_y = int(cornersObj[i][0][0][0]), int(cornersObj[i][0][0][1])
                cv2.putText(frame, f"ID:{current_id}", (c_x, c_y - 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"X:{xmoy:.0f} Y:{ymoy:.0f}", (c_x, c_y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    else:
        # Optionnel : Nettoyer l'historique si aucun objet n'est vu pendant longtemps
        # Ici on ne fait rien pour garder la dernière position connue en mémoire si besoin
        pass

    if not plan_detecte:
        cv2.putText(frame, "Plan non detecte - Placez les 4 coins", (30, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

# --- Écriture JSON différée ---
    # On écrit seulement si frame_count est un multiple de SAVE_INTERVAL
    if frame_count % SAVE_INTERVAL == 0:
        json_data = {
            "frame": frame_count,
            "objects": current_frame_data
        }
        try:
            with open(json_filename, "w") as f:
                json.dump(json_data, f)
            # Petit indicateur visuel de sauvegarde (cercle rouge en haut à droite)
            cv2.circle(frame, (1900, 30), 10, (0, 0, 255), -1) 
        except Exception as e:
            print(f"Erreur écriture JSON: {e}")
            
    cv2.imshow("Suivi Multi-Objets sur Plan", frame)
    frame_count += 1

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()