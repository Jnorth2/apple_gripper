import numpy as np

def net_air_pressure(air_pressures):
    """
    This method takes the pressure readings, assigns them to the air-pressure unit vector and
    obtaind the net_air_pressure vy vector addition
    """

    # Suction-Cup angular location
    scA_loc = + np.pi/3
    scB_loc = + np.pi
    scC_loc = - np.pi/3

    # Air-pressure Unit Vectors
    uA = np.array([np.cos(scA_loc), np.sin(scA_loc), 0])
    uB = np.array([np.cos(scB_loc), np.sin(scB_loc), 0])
    uC = np.array([np.cos(scC_loc), np.sin(scC_loc), 0])

    # Air-pressure vectors
    vA = air_pressures[0] * uA
    vB = air_pressures[1] * uB
    vC = air_pressures[2] * uC

    vSum = vA + vB + vC

    return vSum



def axis_of_rotation(sum_vector):
    """
    The function returns a unit vector that represents an axis of rotation perpendicular
    to the given vector v_sum. This is often used in applications involving rotations, 
    such as robotics or computer graphics, where you need to define a rotation around a specific axis.
    """

    # Approach 1 from Olivias
    # omega = np.array([1., 1., 0. ])
    # omega -= omega.dot(sum_vector) * sum_vector / np.linalg.norm(sum_vector)**2
    # omega /= np.linalg.norm(omega)   #gripper frame

    # Approach 2 from Alejo
    theta = np.arctan2(sum_vector[1], sum_vector[0])

    theta -= np.pi/2          # rotate -90deg

    return theta


def axis_angle_rotation(air_pressures):
    """This method obtains an axis-angle representation from the three air-pressure readings
    returns:    rotation_axis (rads)

    """
    
    LOWEST_VACUUM = 223         # Reference Vacuum [hPa]
    ENGAGE_THRESHOLD = 400      # Engagement reference [HPa]

    # Obtain addition vector    
    # air_pressures = [air_press_A, air_press_B, air_press_C]
    vsum = net_air_pressure(air_pressures)

    rotation_magnitude = np.linalg.norm(vsum)         # Norm of the vector
    rotation_axis = axis_of_rotation(vsum)
    
    # If all engaged, no need to rotate 
    if air_pressures[0] < ENGAGE_THRESHOLD and air_pressures[1] < ENGAGE_THRESHOLD and air_pressures[2] < ENGAGE_THRESHOLD:
        print('All 3 suction cups are engaged')
        rotation_magnitude = 0
        rotation_axis = 0

    return rotation_axis, rotation_magnitude


def center_of_rotation(air_pressures):
    """
    Method to translate the center of rotation to the intersection between (i) the line that connects
    the suction cups engaged and (ii) a line aligned with vsum.
    """
    pA = air_pressures[0]
    pB = air_pressures[1]
    pC = air_pressures[2]

    ENGAGE_THRESHOLD = 400
    SCUP_DISTANCE_TO_CENTER = 0.032     # [m] Distance from the center of the gripper to the center of the cup
    
    x = 0
    y = 0

    # Case A
    if pA < ENGAGE_THRESHOLD and pB > ENGAGE_THRESHOLD and pC > ENGAGE_THRESHOLD:
        x = np.cos(np.pi / 3) * SCUP_DISTANCE_TO_CENTER
        y = np.sin(np.pi / 3) * SCUP_DISTANCE_TO_CENTER
    
    # Case B
    if pB < ENGAGE_THRESHOLD and pA > ENGAGE_THRESHOLD and pC > ENGAGE_THRESHOLD:
        x = -SCUP_DISTANCE_TO_CENTER
        y = 0
    # Case C
    if pC < ENGAGE_THRESHOLD and pB > ENGAGE_THRESHOLD and pA > ENGAGE_THRESHOLD:
        x = np.cos(-np.pi / 3) * SCUP_DISTANCE_TO_CENTER
        y = np.sin(-np.pi / 3) * SCUP_DISTANCE_TO_CENTER
    
    
    # Case A & B
    if pA < ENGAGE_THRESHOLD and pB < ENGAGE_THRESHOLD and pC > ENGAGE_THRESHOLD:
        x = (1 / 2) * ((pA + pC - (2 * pB)) / (pA - (2 * pC) + pB)) * SCUP_DISTANCE_TO_CENTER
        y = (np.sqrt(3) / 2) * ((pA - pC) / (pA - (2 * pC) + pB)) * SCUP_DISTANCE_TO_CENTER
    # Case A & C
    if pA < ENGAGE_THRESHOLD and pC < ENGAGE_THRESHOLD and pB > ENGAGE_THRESHOLD:
        x = (1 / 2) * SCUP_DISTANCE_TO_CENTER
        y = (np.sqrt(3) / 2) * ((pA - pC) / (pA + pC - (2 * pB))) * SCUP_DISTANCE_TO_CENTER
    # Case B & C
    if pB < ENGAGE_THRESHOLD and pC < ENGAGE_THRESHOLD and pA > ENGAGE_THRESHOLD:
        x = (-1 / 2) * ((pA + pC - (2 * pB)) / ((2 * pA) - pC - pB)) * SCUP_DISTANCE_TO_CENTER
        y = (-np.sqrt(3) / 2) * ((pA - pC) / ((2 * pA) - pC - pB)) * SCUP_DISTANCE_TO_CENTER


    return round(x, 4), round(y, 4)     # Units in [m]



def main():
    pass



if __name__ == '__main__':
    main()