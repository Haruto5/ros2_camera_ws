import cv2
import threading
import math

import json

with open('settings.json', 'r') as file:
    settings = json.load(file)

def get_pitch():
    while True:
        camera_height=input("カメラまでの高さを入力(mm)(何も入力がなければsettings.jsonを参照):")
        if(camera_height==""):
            camera_height=settings["camera"]["height"]
        else:
            try:
                camera_height=float(camera_height)
            except:
                print("数字を入力するか、setting.jsonを参照する場合何も入力しないでください")
                continue
        try:
            camera_dist=float(input("カメラの真下からビデオにある交点までの距離を入力(mm):"))
        except:
            print("数字を入力してください")
            continue
        pitch=math.atan(camera_height/camera_dist)*180/math.pi
        settings["camera"]["height"]=camera_height
        settings["camera"]["pitch"]=pitch
        with open('settings.json', 'w') as file:
            json.dump(settings, file, indent=4)

        print("\n高さ:",camera_height,"\nピッチ:",pitch,"\nをsettings.jsonに書き込みました")


cap = cv2.VideoCapture(settings["camera"]["number"], cv2.CAP_DSHOW)
# cap.set(cv2.CAP_PROP_SETTINGS, 1) # カメラ設定ウィンドウ表示

cap.set(cv2.CAP_PROP_FPS, 30)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 270)

t=threading.Thread(target=get_pitch)
t.start()

while True: # 永久ループ
    ret, video = cap.read() # 画像取得
    
    if ret:
        
        height,width=video.shape[:2]

        # 線の色（BGR形式）と太さを設定
        color = (255, 255, 255)  # 緑色
        thickness = 1

        # 線を描画
        cv2.line(video, (width//2, 0), (width//2, height), color, thickness)
        cv2.line(video, (0, height//2), (width, height//2), color, thickness)
        
        
        # 編集されたビデオフレームを表示
        cv2.imshow('video', video)
    
    key = cv2.waitKey(10) # ESCで終了
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
