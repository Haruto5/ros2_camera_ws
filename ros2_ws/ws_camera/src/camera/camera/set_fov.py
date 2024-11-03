import math
import json

with open('settings.json', 'r') as file:
    settings = json.load(file)

def calculate_hfov_vfov(dfov, aspect_ratio):
    # dFOVを度からラジアンに変換
    dfov_rad = math.radians(dfov)
    
    # hFOVとvFOVを計算
    hfov_rad = 2 * math.atan(math.tan(dfov_rad / 2) * math.sqrt(1 / (1 + (1 / aspect_ratio**2))))
    vfov_rad = 2 * math.atan(math.tan(dfov_rad / 2) * math.sqrt(1 / (1 + aspect_ratio**2)))
    
    # ラジアンから度に変換
    hfov_deg = math.degrees(hfov_rad)
    vfov_deg = math.degrees(vfov_rad)
    
    return hfov_deg, vfov_deg


# 例: dFOVが100度、アスペクト比が16:9の場合
aspect_ratio = 16 / 9

dfov = float(input("dFOV(対角視野角)を入力"))
hfov, vfov = calculate_hfov_vfov(dfov, aspect_ratio)

settings["camera"]["hFov"]=hfov
settings["camera"]["vfov"]=vfov

with open('settings.json', 'w') as file:
    json.dump(settings, file, indent=4)


print(f"hFOV: {hfov}\nvFOV: {vfov}\nこれをsettings.jsonに書き込みました。")
