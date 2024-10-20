from air_functions import * 


pa = 200
pb = 951.7
pc = 945.6

axis, mag = axis_angle_rotation([pa, pb, pc])
# axis = round(axis*180/3.1416, 2)
# mag = round(mag*180/3.1416, 2)
print('axis and magnitude: ', axis, round(mag,2))

x,y = center_of_rotation([pa, pb, pc])
print('cr coordinates: ', x, y)


# [950.0, 951.4, 945.3
# 950.1, 951.7, 945.6]
