import cv2
import numpy as np
import glob
import os

# Parámetros del tablero de ajedrez
CHESSBOARD_SIZE = (9, 12)  # Número de esquinas internas (filas, columnas)
SQUARE_SIZE = 0.015  # Tamaño de cada cuadrado en metros (ajustar según tu tablero)

# Preparar puntos 3D del patrón (coordenadas del tablero en 3D)
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2) * SQUARE_SIZE

# Listas para almacenar puntos 3D y puntos 2D de la imagen
objpoints = []  # Puntos en el espacio real 3D
imgpoints = []  # Puntos en el plano de la imagen 2D

# Cargar imágenes del tablero de ajedrez
folder = "/home/ruben/Documents/grass_drone_ros2/src/grass_drone_camera/grass_drone_camera/Calibration_Photos"
images = glob.glob(os.path.join(folder, "*.png"))

print(f"Se encontraron {len(images)} imágenes en {folder}")
if not images:
    print("ERROR: No se encontraron imagenes. Verifica la ruta y el formato de los archivos.")
    exit()

if not images:
    print(f"No se encontraron imágenes en {folder}. Verifica la ruta.")
    exit()


for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detectar esquinas del tablero
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    if ret:
        print(f"Esquinas detectadas en {fname}")
        objpoints.append(objp)  # Guardar puntos 3D
        imgpoints.append(corners)  # Guardar puntos 2D

        # Dibujar y mostrar las esquinas detectadas
        cv2.drawChessboardCorners(img, CHESSBOARD_SIZE, corners, ret)
        cv2.imshow("Chessboard", img)
        cv2.waitKey(500)
    else:
        print(f"No se detectaron esquinas en {fname}")
        

cv2.destroyAllWindows()

if not imgpoints:
    print("ERROR: No se detectaron esquinas en ninguna imagen. Verifica las fotos y el tamaño del patrón.")
    exit()


# Calibrar la cámara
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Guardar los parámetros en un archivo
np.save("camera_matrix.npy", camera_matrix)
np.save("dist_coeffs.npy", dist_coeffs)

print("Camera Matrix:\n", camera_matrix)
print("\nDistortion Coefficients:\n", dist_coeffs)