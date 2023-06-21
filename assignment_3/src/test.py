import math

P_ENTRY = (1036, 162)
P_END = (1129, 69)

delta_x = P_ENTRY[0] - P_END[0]
delta_y = P_ENTRY[1] - P_END[1]

angle_rad = math.atan2(delta_y, delta_x)
angle_deg = math.degrees(angle_rad)

if angle_deg < 0:
    angle_deg += 360

print("두 좌표 사이의 각도: ", angle_deg)
