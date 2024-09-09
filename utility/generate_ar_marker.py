import cv2
import numpy as np

# arucoライブラリの辞書を定義
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

# IDが0のマーカを生成
marker_id = 0
marker_size = 1000  # マーカのサイズ（ピクセル）
marker_img = np.zeros((marker_size, marker_size), dtype=np.uint8)
marker_img = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size, marker_img, 1)

# マーカのサイズを10cmにするためのピクセル数（DPIを100とする）
dpi = 100
cm_to_inch = 2.54
marker_size_cm = 10
marker_size_inch = marker_size_cm / cm_to_inch
marker_size_pixel = int(dpi * marker_size_inch)

# 余白のサイズ（2cm）
margin_cm = int(marker_size_cm) * 0.2 # 20%の余白
margin_inch = margin_cm / cm_to_inch
margin_pixel = int(dpi * margin_inch)

# ボックスの辺の長さ
box_size_cm = margin_cm + marker_size_cm
box_size_inch = box_size_cm / cm_to_inch
box_size_pixel = int(dpi * box_size_inch)

# 白いキャンバスを作成
canvas_width = 4 * box_size_pixel
canvas_height = 3 * box_size_pixel
canvas = 255 * np.ones((canvas_width, canvas_height), dtype=np.uint8)

# マーカーをリサイズして余白を設ける
resized_marker = cv2.resize(marker_img, (marker_size_pixel, marker_size_pixel))
marker_with_margin = 255 * np.ones((box_size_pixel, box_size_pixel), dtype=np.uint8)
start_point = (box_size_pixel - marker_size_pixel) // 2
marker_with_margin[start_point:start_point + marker_size_pixel, start_point:start_point + marker_size_pixel] = resized_marker

# 中央の一面（展開図の中央）にマーカーを配置
canvas[box_size_pixel:2 * box_size_pixel, box_size_pixel:2 * box_size_pixel] = marker_with_margin

# 画像を保存
output_file = '../texture/ar_marker_box.png'
cv2.imwrite(output_file, canvas)

print(f"立方体の展開図を{output_file}に保存しました。")