from math import (pi, sin, cos, atan2,
                  radians as rad,
                  degrees as deg)

a = {
    "j1_left" : [-rad(45), rad(30)],
    "j1_right" : [-rad(30), rad(45)],
    "j2" : [0, 0],
    "j3" : [0, 0],
    "j4" : [0, 0],
    "j5" : [0, 0],
}

parallel_shit = atan2(84.5, 415)

a["j2"][1] = pi - parallel_shit
a["j2"][0] = a["j2"][1] - rad(90-1.6)

a["j3"][0] = -a["j2"][1]
a["j3"][1] = a["j3"][0] + pi

a["j4"] = [-rad(150), rad(150)]
a["j5"] = [-rad(100), rad(100)]

for k, v in a.items():
    print(f"{k}: {v}")
