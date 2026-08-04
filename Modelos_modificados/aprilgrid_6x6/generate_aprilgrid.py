#!/usr/bin/env python3
"""
Genera:
  1. aprilgrid_6x6.png   -> textura para el modelo Gazebo
  2. aprilgrid_6x6.json  -> config del target para basalt_calibrate

Las dimensiones físicas (metros) y en píxeles se derivan de las MISMAS
variables, por lo que la imagen y el json son consistentes por construcción.
"""
import cv2
import numpy as np
import json

# ---- Parámetros del target (ajustar aquí si se quiere otro tamaño) ----
TAG_COLS = 6
TAG_ROWS = 6
TAG_SIZE_M = 0.08          # lado del cuadrado negro de cada tag, en metros
TAG_SPACING_RATIO = 0.3    # hueco entre tags, como fracción de TAG_SIZE_M
PX_PER_M = 2000            # resolución de la textura (px por metro)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

gap_m = TAG_SPACING_RATIO * TAG_SIZE_M
border_m = gap_m  # margen exterior igual al hueco entre tags

tag_px = int(round(TAG_SIZE_M * PX_PER_M))
gap_px = int(round(gap_m * PX_PER_M))
border_px = int(round(border_m * PX_PER_M))

width_m = TAG_COLS * TAG_SIZE_M + (TAG_COLS - 1) * gap_m + 2 * border_m
height_m = TAG_ROWS * TAG_SIZE_M + (TAG_ROWS - 1) * gap_m + 2 * border_m

width_px = TAG_COLS * tag_px + (TAG_COLS - 1) * gap_px + 2 * border_px
height_px = TAG_ROWS * tag_px + (TAG_ROWS - 1) * gap_px + 2 * border_px

# Fondo blanco (quiet zone requerida por el detector AprilTag)
canvas = np.ones((height_px, width_px), dtype=np.uint8) * 255

# Convención de numeración: fila 0 = fila superior de la imagen,
# id = row * TAG_COLS + col  (igual que se referenciará en el json)
tag_id = 0
for row in range(TAG_ROWS):
    for col in range(TAG_COLS):
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, tag_px)
        y0 = border_px + row * (tag_px + gap_px)
        x0 = border_px + col * (tag_px + gap_px)
        canvas[y0:y0 + tag_px, x0:x0 + tag_px] = marker
        tag_id += 1

cv2.imwrite("aprilgrid_6x6.png", canvas)

config = {
    "tagFamily": "tag36h11",
    "tagCols": TAG_COLS,
    "tagRows": TAG_ROWS,
    "tagSize": TAG_SIZE_M,
    "tagSpacing": TAG_SPACING_RATIO,
    "boardWidthMeters": round(width_m, 6),
    "boardHeightMeters": round(height_m, 6),
    "note": "tagId = row * tagCols + col, fila 0 arriba, tal como se ha renderizado en aprilgrid_6x6.png"
}
with open("aprilgrid_6x6.json", "w") as f:
    json.dump(config, f, indent=2)

print(f"Panel: {width_m:.4f} x {height_m:.4f} m  ({width_px} x {height_px} px)")
print(f"Tag size: {TAG_SIZE_M} m | gap: {gap_m:.4f} m | border: {border_m:.4f} m")
