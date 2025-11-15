import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
import numpy as np

import json

# --- PARAMÈTRES ---
aruco_dict_board = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_dict_object = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

plan_marker_length = 30.0   # mm
plan_ids = [1, 2, 3, 4]     # IDs des coins
object_marker_length = 18.0 # mm
object_height = 38.0         # mm
REFERENCE_Z = "dessus"      # "dessus", "centre" ou "dessus"
json_filename = "positions.json"
 
# --- Zoom sur l'objet détecté ---
zoom_factor = 3.0
zoom_size = 100


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

positions = []
frame_count = 0


# ==============================================
# FILTRE DE KALMAN POUR LISSAGE DES COORDONNÉES
# ==============================================
kalman = cv2.KalmanFilter(6, 3)
# État : [x, y, z, vx, vy, vz]
kalman.measurementMatrix = np.array([
    [1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0]
], np.float32)

kalman.transitionMatrix = np.array([
    [1, 0, 0, 1, 0, 0],
    [0, 1, 0, 0, 1, 0],
    [0, 0, 1, 0, 0, 1],
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1]
], np.float32)

kalman.processNoiseCov = np.eye(6, dtype=np.float32) * 1e-3
kalman.measurementNoiseCov = np.eye(3, dtype=np.float32) * 1e-1
kalman.errorCovPost = np.eye(6, dtype=np.float32)
kalman.statePost = np.zeros((6, 1), np.float32)


print("=== Suivi ArUco - Détection automatique du plan ===")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners4x4, ids4x4, _ = cv2.aruco.detectMarkers(gray, aruco_dict_board)
    cornersObj, idsObj, _ = cv2.aruco.detectMarkers(gray, aruco_dict_object)

    cv2.aruco.drawDetectedMarkers(frame, corners4x4, ids4x4)
    cv2.aruco.drawDetectedMarkers(frame, cornersObj, idsObj, (0, 255, 0))

    # --- Détermination automatique du plan ---
    rvec_board = None
    tvec_board = None
    largeur = hauteur = None
    plan_detecte = False

    if ids4x4 is not None and len(ids4x4) >= 4:
        corners_3d = {}

        for i, id_ in enumerate(ids4x4.flatten()):
            if id_ in plan_ids:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [corners4x4[i]], plan_marker_length, camera_matrix, dist_coeffs
                )
                corners_3d[int(id_)] = tvecs[0][0]
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                                  rvecs[0], tvecs[0], plan_marker_length / 2)

        if all(id_ in corners_3d for id_ in plan_ids):
            plan_detecte = True
            P1, P2, P3, P4 = [np.array(corners_3d[i]) for i in plan_ids]
#            largeur = np.linalg.norm(P2 - P1)
#            hauteur = np.linalg.norm(P1 - P4)
            hauteur = np.linalg.norm(P2 - P1)
            largeur = np.linalg.norm(P1 - P4)
            centre_plan = (P1 + P2 + P3 + P4) / 4.0
            v1, v2 = P2 - P1, P4 - P1
            normale = np.cross(v1, v2)
            normale /= np.linalg.norm(normale)

            cv2.putText(frame, f"Plan: {largeur:.1f}x{hauteur:.1f} mm",
                        (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)

    # --- Détection de l'objet ---
    rvec_obj = None
    tvec_obj = None

    if idsObj is not None and len(idsObj) > 0:
        rvecs_obj, tvecs_obj, _ = cv2.aruco.estimatePoseSingleMarkers(
            cornersObj, object_marker_length, camera_matrix, dist_coeffs
        )
        rvec_obj = rvecs_obj[0]
        tvec_obj = tvecs_obj[0]

        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec_obj, tvec_obj, 40)

        if plan_detecte:
            # --- Axes du repère du plan ---
#            X_axis = (P2 - P1)
#            X_axis /= np.linalg.norm(X_axis)
#            Y_axis = (P4 - P1)
#            Y_axis /= np.linalg.norm(Y_axis)

            X_axis = (P4 - P1)
            X_axis /= np.linalg.norm(X_axis)

            Y_axis = (P2 - P1)
            Y_axis /= np.linalg.norm(Y_axis)

            # Normale au plan (vers la caméra)
            Z_axis = np.cross(X_axis, Y_axis)
            Z_axis /= np.linalg.norm(Z_axis)

            # Matrice de rotation plan->caméra
            R_board = np.column_stack((X_axis, Y_axis, Z_axis))
            R_inv = R_board.T  # caméra -> plan

            # Position de l'objet (centre du marqueur) dans le repère caméra
            pos_cam_obj = tvec_obj.reshape(3, 1)
            pos_plan_origin = centre_plan.reshape(3, 1)

            # --- Position projetée dans le repère du plan ---
            pos_obj_plan = np.dot(R_inv, (pos_cam_obj - pos_plan_origin))

            # --- Calcul de la distance géométrique au plan (hauteur réelle) ---
            # Z = projection de (P_obj - P_plan) sur la normale
            z_geo = np.dot(Z_axis, (pos_cam_obj.flatten() - centre_plan))

            # Position dans le repère du plan (X,Y = dans le plan, Z = perpendiculaire)
            x, y = pos_obj_plan[0, 0], pos_obj_plan[1, 0]
            z = z_geo  # distance perpendiculaire réelle

            # --- Ajustement selon la référence verticale ---
            z_offset = 0.0  # tu peux mettre 5 ou 10 mm pour un petit décalage empirique
            if REFERENCE_Z == "centre":
                z -= object_height / 2
            elif REFERENCE_Z == "sol":
                z -= object_height
            z -= z_offset

            # --- Moyenne mobile sur les dernières positions ---
            positions.append((x, y, z))
            if len(positions) > 10:
                positions.pop(0)

            xmoy = np.mean([p[0] for p in positions])
            ymoy = np.mean([p[1] for p in positions])
            zmoy = np.mean([p[2] for p in positions])

            data = {"frame":frame_count,"x":round(x, 1),"y":round(y, 1),"z":round(z, 1),"xmoy":round(xmoy, 1),"ymoy":round(ymoy, 1),"zmoy":round(zmoy, 1)}

            with open(json_filename,"w") as f:
               json.dump(data,f)

            # --- Affichage ---
            cv2.putText(frame,
                        f"Objet (plan): X={x:.1f}  Y={y:.1f}  Z={z:.1f}",
                        (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame,
                        f"Moyenne: XM={xmoy:.1f}  YM={ymoy:.1f}  ZM={zmoy:.1f}",
                        (30, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
    else:
        cv2.putText(frame, "Objet non detecte", (30, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow("Suivi Objet sur Plan", frame)

    frame_count += 1

    if cv2.waitKey(1) & 0xFF == 27:
        break


cap.release()
cv2.destroyAllWindows()
