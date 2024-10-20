import numpy as np
import math

theta = math.radians(150)
ROT_MATRIX = np.array([[np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta),  np.cos(theta), 0],
                [0,0,1]])



speeds = np.array([-0.1, 0, 0])
angular_velocities = np.dot(ROT_MATRIX, speeds.T)

position_vector = np.array([0, 0, -0.2])
linear_velocities = np.cross(angular_velocities, position_vector)

print(angular_velocities, linear_velocities)
      