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

def wedge_of_axes(air_pressures, d_theta = 1, wedge_angle = 60, gripper_height = 0.19):
    """
    This function optains the wedge of possible axis to rotate the gipper which will rotate towards engaging 
    at least one more suction cup. 

    The width of the wedge is determined by the number of suction cups engaged. For starters this will be a 
    margin around the best axis of rotation, however it could also be the biaxis between two suction cups if 
    only one is engaged, or the axis at one suction if two are engaged. We will provide a wedge of plus or 
    minus 60 degrees for when one suction cup is engaged, and a wedge of plus or minus 30 degrees when two 
    suction cups are engaged. 

    Parameters
    ----------
    air_pressures : list
        A list containing the air pressure readings from the suction cups.
    d_theta : float
        Angular spacing between consecutive axes in the wedge [deg].
    wedge_angle : float
        The total angle of the wedge [deg].
    gripper_height : float
        Distance from 'tool0' to the plane of the suction cups [m] (GRIPPER_HEIGHT in grasp_controller).


    Returns
    -------
    wedge : numpy array
        A 2D array containing a descrete wedge of unit twists.
        Shape (N, 6): rows [vx, vy, vz, wx, wy, wz] = [v; w], the twist of 'tool0' for a rotation of
        1 rad/s about each candidate axis through the center of rotation (same pivot as
        center_of_rotation). Components are expressed in the 'gripper_scups_link' axes, so rotate both
        v and w into the Jacobian's frame before use. Ordered from -half_width to +half_width, so the
        best axis is the middle row. Shape (0, 6) when zero or all three suction cups are engaged
        (no wedge defined).

    """
    ENGAGE_THRESHOLD = 400      # Engagement reference [HPa]
    #get best rotation axis
    vsum = net_air_pressure(air_pressures)
    axis = axis_of_rotation(vsum)

    # Wedge margin
    n_engaged = sum(p < ENGAGE_THRESHOLD for p in air_pressures)
    if n_engaged == 1:
        half_width = np.radians(wedge_angle)
    elif n_engaged == 2:
        half_width = np.radians(wedge_angle / 2)
    else:
        return np.empty((0, 6))

    # Discretize the wedge of axes
    n_axes = int(round(2 * half_width / np.radians(d_theta))) + 1
    angles = axis + np.linspace(-half_width, half_width, n_axes)

    # Unit angular velocities
    w = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(n_axes)))

    # Linear velocity so the center of rotation stays fixed
    cr_x, cr_y = center_of_rotation(air_pressures)
    position_vector = np.array([-cr_x, -cr_y, -gripper_height])
    v = np.cross(w, position_vector)

    wedge = np.hstack((v, w))

    return wedge

def weight_wedge_pressure(air_pressures, wedge, weight_type = 'linear', scale = None):
    """
    This function weights the wedge of possible axes by the best axis of rotation to engage the remaining
    suction cups.

    Parameters
    ----------
    air_pressures : list
        A list containing the air pressure readings from the suction cups.
    wedge : numpy array
        An array containing the descretized wedge of unit twists [Nx6].
    weight_type : str
        'linear'   : w = max(0, 1 - |d| / scale)        (tent, like -abs())
        'gaussian' : w = exp(-d^2 / (2 * scale^2))      (scale = standard deviation)
        'laplace'  : w = exp(-|d| / scale)              (scale = diversity b)
        where d is the angle between each axis and the best axis.
    scale : float or None
        Spread of the weighting [deg]. If None it is taken from the wedge half-width: linear reaches 0 at
        the wedge edges, gaussian and laplace use half-width / 2 (both give exp(-2) ~ 0.14 at the edges).

    Returns
    -------
    weights : numpy array
        A 1D array contatining the weights for each axis in the wedge.
        Unit weights in [0, 1], equal to 1 at the best axis, so a gain can be applied later.
        Empty when the wedge is empty.
    """
    if len(wedge) == 0:
        return np.empty(0)

    # Best axis of rotation
    vsum = net_air_pressure(air_pressures)
    axis = axis_of_rotation(vsum)

    # Angle of each axis in the wedge from the best axis, wrapped to [-pi, pi)
    angles = np.arctan2(wedge[:, 4], wedge[:, 3])
    d = np.abs((angles - axis + np.pi) % (2 * np.pi) - np.pi)

    # Spread of the weighting
    half_width = np.max(d)
    if scale is None:
        scale = half_width if weight_type == 'linear' else half_width / 2
    else:
        scale = np.radians(scale)

    if weight_type == 'linear':
        weights = np.clip(1 - d / scale, 0, 1)
    elif weight_type == 'gaussian':
        weights = np.exp(-d**2 / (2 * scale**2))
    elif weight_type == 'laplace':
        weights = np.exp(-d / scale)
    else:
        raise ValueError(f"weight_type must be 'linear', 'gaussian' or 'laplace', got '{weight_type}'")

    return weights


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