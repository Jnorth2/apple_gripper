from air_functions import * 


pa = 200
pb = 300
pc = 200

axis, mag = axis_angle_rotation(pa, pb, pc)
axis = round(axis*180/3.1416, 2)
print('axis and magnitude: ', axis, round(mag,2))

x,y = center_of_rotation(pa, pb, pc)
print('cr coordinates: ', x, y)

