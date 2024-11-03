import numpy as np
import cv2
import math
import time
import json
import sys

debug=False


with open('settings.json', 'r') as file:
    settings = json.load(file)

for i in range(1,len(sys.argv)):
    argument=sys.argv[i]
    if argument=="debug":
        debug=True

# 視野角とカメラピッチ角をラジアンに変換
hFov_rad = settings["camera"]["hFov"] * math.pi / 180
vFov_rad = settings["camera"]["vFov"] * math.pi / 180
camera_pitch_rad = settings["camera"]["pitch"] * math.pi / 180

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

#総最小二乗法
def total_least_squares(x, y):
    # データを平均中心化
    x_mean = np.mean(x)
    y_mean = np.mean(y)
    X = x - x_mean
    Y = y - y_mean

    # 共分散と分散を計算
    Sxy = np.sum(X * Y)
    Sxx = np.sum(X * X)

    if Sxx == 0:
        return 0,0

    # 直線の傾き a と切片 b を計算
    a = Sxy / Sxx
    b = y_mean - a * x_mean

    return a, b

def getEdge(img):
    height, width = img.shape[:2]
    newImg=img.copy()
    blurred = cv2.GaussianBlur(img, (round_up_to_odd(settings["blurLevel_ratio_to_get_edge"]*width), round_up_to_odd(settings["blurLevel_ratio_to_get_edge"]*height)), 0)
    # 画像をHSV色空間に変換
    hsv_image = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # 白と黒のマスクを作成
    white_mask = cv2.inRange(hsv_image, np.array(settings["color_ranges"]["white"]["lower"]), np.array(settings["color_ranges"]["white"]["upper"]))

    # 輪郭の検出
    contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours)==0:
        if(not debug):
            return None
        return None, newImg,newImg,white_mask,newImg

    # 面積が一番大きい輪郭を見つける
    largest_contour = max(contours, key=cv2.contourArea)

    # 面積を表示
    if(cv2.contourArea(largest_contour)<settings["min_wall_area_ratio"]*height*width):
        if(not debug):
            return None
        return None,newImg,newImg,white_mask,newImg
    
    # 元の画像と同じサイズの黒いマスクを作成
    mask = np.zeros_like(white_mask)
    

    # 面積が一番大きい輪郭を白で塗りつぶす
    cv2.drawContours(mask, [largest_contour], -1, (255), thickness=cv2.FILLED)

    # マスクを使用して元の画像を黒くする
    result = cv2.bitwise_and(white_mask, mask)

    black_mask = cv2.inRange(hsv_image, np.array(settings["color_ranges"]["black"]["lower"]), np.array(settings["color_ranges"]["black"]["upper"]))

    # カーネルを定義（膨張の範囲を決定）
    kernel = np.ones((int(settings["kernel_size"]*width),int(settings["kernel_size"]*height)), np.uint8)

    # 膨張処理
    dilated_white_mask = cv2.dilate(result, kernel, iterations=1)
    dilated_black_mask = cv2.dilate(black_mask, kernel, iterations=1)
    # 白と黒のマスクを組み合わせる
    combined_mask = cv2.bitwise_and(dilated_white_mask, dilated_black_mask)
    if(cv2.countNonZero(combined_mask)<settings["min_line_area_ratio"]*height*width):
        if not debug:
            return None
        return None,newImg,combined_mask,white_mask,black_mask

    white_pixels = np.column_stack(np.where(combined_mask == 255))
    # 白いピクセル（255）の座標を取得
    
    # x, y座標を分ける
    x = white_pixels[:, 1]  # 列インデックスがx座標
    y = white_pixels[:, 0]  # 行インデックスがy座標

    # 総最小二乗法で直線をフィッティング（y = ax + b）
    a, b = total_least_squares(x, y)

    # フィッティングした直線の端点を決める
    x_min, x_max = 0, width
    y_min, y_max = a * x_min + b, a * x_max + b

    leftEdge=get_position_from_point(x_min,y_min)
    rightEdge=get_position_from_point(x_max,y_max)

    if debug:
        # 直線を描画
        cv2.line(newImg, (x_min, int(y_min)), (x_max, int(y_max)), (0, 0, 255), 2)
        return [leftEdge,rightEdge],newImg,combined_mask,white_mask,black_mask
    else:
        return [leftEdge,rightEdge]
    

def getAngle(p1, p2):
    # 二つの点をベクトルに変換
    vector = np.array(p2) - np.array(p1)
    
    # 原点から直線への垂線の足を求める
    t = -np.dot(np.array(p1), vector) / np.dot(vector, vector)
    closest_point = np.array(p1) + t * vector
    
    # 原点から垂線の足へのベクトル
    origin_to_closest = closest_point
    
    # 単位ベクトル[1,0]
    unit_vector = np.array([1, 0])
    
    # ベクトルの内積とノルムを使って角度を求める
    dot_product = np.dot(origin_to_closest, unit_vector)
    norm_origin_to_closest = np.linalg.norm(origin_to_closest)
    norm_unit_vector = np.linalg.norm(unit_vector)
    
    cos_theta = dot_product / (norm_origin_to_closest * norm_unit_vector)
    angle = np.arccos(cos_theta)
    
    # ラジアンから度に変換
    angle_degrees = np.degrees(angle)
    # 角度を-180度から180度の範囲に調整
    if origin_to_closest[1] > 0:
        angle_degrees = -angle_degrees
    
    return angle_degrees



cap = cv2.VideoCapture(settings["camera"]["number"], cv2.CAP_DSHOW) # 0番目のカメラ。DirectShow書かないと動かなかったwindows
cap.set(cv2.CAP_PROP_SETTINGS, 1) # カメラ設定ウィンドウ表示

cap.set(cv2.CAP_PROP_FPS, 30)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 270)

def show(img):
    cv2.imshow('', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
while True: # 永久ループ
    start_time = time.time() # 画像取得前のタイムスタンプ
    ret, video = cap.read() # 画像取得
    height, width = video.shape[:2]
    if ret:
        if debug:
            position, editedImg,combined_mask,white,black = getEdge(video)
        else:
            position=getEdge(video)
        angle=None
        if position!=None:
            angle=getAngle(position[0],position[1])
        
        end_time = time.time() # 画像表示後のタイムスタンプ
        delay = end_time - start_time

        print(angle)

        if debug:  
            # 線の色（BGR形式）と太さを設定
            color = (255, 255, 255)  # 緑色
            thickness = 1

            # 線を描画
            cv2.line(editedImg, (width//2, 0), (width//2, height), color, thickness)
            cv2.line(editedImg, (0, height//2), (width, height//2), color, thickness)
        
            # フレームレートを表示
            cv2.putText(editedImg, f'delay: {int(delay*1000)} ms', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(editedImg, f'angle: {angle} degrees', (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        
            # 編集されたビデオフレームを表示
            cv2.imshow('edited', editedImg)
            cv2.imshow('black', black)
            cv2.imshow('white', white)
            cv2.imshow('combined', combined_mask)
    
    key = cv2.waitKey(10) # ESCで終了
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()