import cv2
import numpy as np
import math
import json
import sys

debug=False
from ament_index_python.packages import get_package_share_directory
import os
def load_settings(package_name: str):
    # パッケージのshareディレクトリからJSONファイルを取得
    share_dir = get_package_share_directory(package_name)
    json_path = os.path.join(share_dir, 'settings.json')

    with open(json_path, 'r') as f:
        settings = json.load(f)

    return settings

settings = load_settings('camera')
print(settings)


for i in range(1,len(sys.argv)):
    argument=sys.argv[i]
    if argument=="debug":
        debug=True

# 視野角とカメラピッチ角をラジアンに変換
hFov_rad = settings["camera"]["hFov"] * math.pi / 180
vFov_rad = settings["camera"]["vFov"] * math.pi / 180
camera_pitch_rad = settings["camera"]["pitch"] * math.pi / 180

def is_merging(x1, y1, w1, h1, x2, y2, w2, h2,height,width):
    return (x1 + w1+settings["merge_distance"]*width >= x2 and x1 <= x2 + w2+settings["merge_distance"]*width 
            and y1 + h1+settings["merge_distance"]*height >= y2 and y1 <= y2 + h2+settings["merge_distance"]*height)

def get_and_remove_rows(matrix, target):
    # 特定の数字を含む行を取得
    rows_with_target = [row for row in matrix if target in row]
    
    # 特定の数字を含む行を削除
    matrix = [row for row in matrix if target not in row]
    
    return rows_with_target, matrix

#離れて見える同じオブジェクトをくっつける(アームが阻害等)
def combineRectangles(rectangles):
    #重なっている組を作成
    combines=[]
    for i in range(len(rectangles)):
        for j in range(i+1,len(rectangles)):
            if is_merging(*rectangles[i],*rectangles[j],height,width):
                combines.append([i,j])
    
    #重なっている組同士をまとめる
    combined=[]
    rect_list=set(range(len(rectangles)))
    while len(rect_list)!=0:
        deleted_targets=set()
        targets={rect_list.pop()}
        while len(targets)!=0:
            target=targets.pop()
            deleted_targets.add(target)
            rect_list-={target}

            removed,combines=get_and_remove_rows(combines,target)
            unique_elements = set()
            for row in removed:
                for element in row:
                    unique_elements.add(element)
            targets.update(unique_elements-deleted_targets)
        combined.append(list(deleted_targets))

    #新しい長方形の作成
    newRectangles=[]
    for indexes in combined:
        min_x=float("inf")
        min_y=float("inf")
        max_x=float("-inf")
        max_y=float("-inf")
        for i in indexes:
            min_x=min(min_x,rectangles[i][0])
            min_y=min(min_y,rectangles[i][1])
            max_x=max(max_x,rectangles[i][0]+rectangles[i][2])
            max_y=max(max_y,rectangles[i][1]+rectangles[i][3])
        newRectangles.append((min_x,min_y,max_x-min_x,max_y-min_y))

    return newRectangles

def round_up_to_odd(number):
    rounded = math.ceil(number)
    if rounded % 2 == 0:
        rounded += 1
    return rounded

def get_position_from_point(x,y):
    global width,height
    # 座標を正規化
    x -= width / 2
    y -= height / 2
    x /= width / 2
    y /= height / 2

    # 角度計算
    yaw_rad = math.atan(x * math.tan(hFov_rad / 2))
    pitch_rad = math.atan(y * math.tan(vFov_rad / 2)) + camera_pitch_rad

    #相対位置計算
    #カメラから見た前方の位置
    relative_positionX = settings["camera"]["height"] / math.tan(pitch_rad)
    #カメラから見た左方向の位置
    relative_positionY = -relative_positionX * math.tan(yaw_rad)
    return [relative_positionX,relative_positionY]

bgr_colors = {
    "red": (0, 0, 255),  # 赤 (BGR形式)
    "green": (0, 255, 0),  # 緑
    "blue": (255, 0, 0),  # 青
    "yellow": (0, 255, 255)  # 黄色
}


# 物体の相対位置を計算
def get_object_position(img):
    global width, height
    newImg = img.copy()
    # 画像の高さと幅を取得
    height, width = img.shape[:2]
    # 画像をぼかす
    blurred = cv2.GaussianBlur(img, (round_up_to_odd(settings["blurLevel_ratio"]*width), round_up_to_odd(settings["blurLevel_ratio"]*height)), 0)
    # 色空間をBGRからHSVに変換
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    positions = []
    
    # 同じサイズの真っ黒な画像を作成
    maskedImg = np.zeros((height, width, 3), dtype=np.uint8)

    for color in settings["block_colors"]:
        # 指定された色のマスクを作成
        mask = None
        if color == "red":
            mask1 = cv2.inRange(hsv, np.array(settings["color_ranges"][color]["lower1"]), np.array(settings["color_ranges"][color]["upper1"]))
            mask2 = cv2.inRange(hsv, np.array(settings["color_ranges"][color]["lower2"]), np.array(settings["color_ranges"][color]["upper2"]))
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, np.array(settings["color_ranges"][color]["lower"]), np.array(settings["color_ranges"][color]["upper"]))
            
        # マスクが適用された領域に対応する色をBGR形式で適用
        maskedImg[mask > 0] = bgr_colors[color]

        # マスクの面積を計算
        area = cv2.countNonZero(mask)
        if area < height * width * settings["min_area_ratio"]:
            continue

        # 輪郭の検出
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        rectangles = []

        for contour in contours:
            # 輪郭の面積を計算
            area = cv2.contourArea(contour)
            # 面積がmin_pixels以上の場合のみ物体として認識
            if area < settings["min_area_ratio"] * height * width:
                continue

            # 輪郭に基づいてバウンディングボックスを取得
            rectangles.append(list(cv2.boundingRect(contour)))

        # 一つの物体だったらくっつける
        newRectangles = combineRectangles(rectangles)

        for x, y, w, h in newRectangles:    
            

            # 写真から見た物体が置いてある位置を推定
            cX = x + w / 2
            cY = y + h * 0.8

            relative_position = get_position_from_point(cX, cY)
            
            positions+=[settings["block_colors"].index(color)]+relative_position

            if not debug:
                continue
            ### 動作確認用
            # 長方形を描く
            start_point = (x, y)  # 長方形の左上の頂点
            end_point = (x + w, y + h)  # 長方形の右下の頂点
            fill_color = bgr_colors[color]  # 長方形の色を指定された色に変更
            thickness = 2  # 長方形の線の太さ

            # 長方形を描画
            cv2.rectangle(newImg, start_point, end_point, fill_color, thickness)

            # 円を描く
            center_coordinates = (int(x + w / 2), int(y + h * 0.8))  # 円の中心座標
            radius = 5  # 円の半径
            fill_color = (255, 255, 255)  # 円の色 (BGR形式)
            thickness = 2  # 円の線の太さ

            # 円を描画
            cv2.circle(newImg, center_coordinates, radius, fill_color, thickness)

            cv2.putText(newImg, f"x:{relative_position[0]:.2f} y:{relative_position[1]:.2f}", start_point, cv2.FONT_HERSHEY_SIMPLEX, 0.5, fill_color, 1)
            ### 
         
    if debug:
        return positions, maskedImg, newImg
    return positions

def get_position():
    ret, video = cap.read() # 画像取得
    if ret:
        position=get_object_position(video)
        return position

    return []    




cap = cv2.VideoCapture(settings["camera"]["number"])

def show(img):
    cv2.imshow('', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
