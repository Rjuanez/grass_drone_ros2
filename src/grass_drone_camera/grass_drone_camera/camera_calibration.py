import cv2
import numpy as np
import glob

# Definir dimensiones del tablero de ajedrez
CHESSBOARD_SIZE = (7, 10)  # Cambia según el tamaño del tablero
SQUARE_SIZE = 0.025  # 2.5 cm por cuadro, ajusta según el tamaño real

# Criterios de terminación
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Puntos 3D del tablero
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2) * SQUARE_SIZE

# Listas para almacenar puntos
objpoints = []  # Puntos 3D en el mundo real
imgpoints = []  # Puntos 2D en la imagen

# Captura desde la cámara
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        cv2.drawChessboardCorners(frame, CHESSBOARD_SIZE, corners2, ret)

    cv2.imshow("Calibración de Cámara", frame)

    if cv2.waitKey(1) & 0xFF == ord('c'):
        break

cap.release()
cv2.destroyAllWindows()

# Calibrar la cámara
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Matriz de la cámara:")
print(camera_matrix)
print("\nCoeficientes de distorsión:")
print(dist_coeffs)