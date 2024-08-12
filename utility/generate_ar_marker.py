import cv2
import numpy as np

# arucoライブラリの辞書を定義
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

# IDが0のマーカを生成
markerId = 0
markerSize = 1000  # マーカのサイズ（ピクセル）
markerImg = np.zeros((markerSize, markerSize), dtype=np.uint8)
markerImg = cv2.aruco.drawMarker(arucoDict, markerId, markerSize, markerImg, 1)

# マーカのサイズを10cmにするためのピクセル数（DPIを100とする）
dpi = 100
cmToInch = 2.54
markerSizeCm = 10
markerSizeInch = markerSizeCm / cmToInch
markerSizePixel = int(dpi * markerSizeInch)

# 余白のサイズ（2cm）
marginCm = int(markerSizeCm) * 0.2 # 20%の余白
marginInch = marginCm / cmToInch
marginPixel = int(dpi * marginInch)

# ボックスの辺の長さ
boxSizeCm = marginCm + markerSizeCm
boxSizeInch = boxSizeCm / cmToInch
boxSizePixel = int(dpi * boxSizeInch)

# 白いキャンバスを作成
canvasWidth = 4 * boxSizePixel
canvasHeight = 3 * boxSizePixel
canvas = 255 * np.ones((canvasWidth, canvasHeight), dtype=np.uint8)

# マーカーをリサイズして余白を設ける
resizedMarker = cv2.resize(markerImg, (markerSizePixel, markerSizePixel))
markerWithMargin = 255 * np.ones((boxSizePixel, boxSizePixel), dtype=np.uint8)
startPoint = (boxSizePixel - markerSizePixel) // 2
markerWithMargin[startPoint:startPoint + markerSizePixel, startPoint:startPoint + markerSizePixel] = resizedMarker

# 中央の一面（展開図の中央）にマーカーを配置
canvas[boxSizePixel:2 * boxSizePixel, boxSizePixel:2 * boxSizePixel] = markerWithMargin

# 画像を保存
outputFile = '../texture/ar_marker_box.png'
cv2.imwrite(outputFile, canvas)

print(f"立方体の展開図を{outputFile}に保存しました。")