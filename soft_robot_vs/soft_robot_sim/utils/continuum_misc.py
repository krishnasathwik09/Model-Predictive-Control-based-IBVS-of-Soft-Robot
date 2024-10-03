import numpy as np

def continuum_fk_arc(s, kappa, phi, ksi=1):
    """
    Get Transformation Matrix by given (s, kappa, phi)
    """
    theta = ksi*s*kappa
    phi = np.take(phi, 0)
    R11 = np.cos(phi)*np.cos(phi)*np.cos(theta) + np.sin(phi)*np.sin(phi)
    R12 = np.cos(phi)*np.sin(phi)*(np.cos(theta)-1)
    R13 = np.cos(phi)*np.sin(theta)
    R21 = R12
    R22 = np.sin(phi)*np.sin(phi)*np.cos(theta) + np.cos(phi)*np.cos(phi)
    R23 = np.sin(phi)*np.sin(theta)
    R31 = -R13
    R32 = -R23
    R33 = np.cos(theta)

    if kappa != 0:
        PX = np.cos(phi)*(1-np.cos(theta))/kappa
        PY = np.sin(phi)*(1-np.cos(theta))/kappa
        PZ = np.sin(theta)/kappa
    else:
        PX = 0
        PY = 0
        PZ = s

    T = np.array([[R11, R12, R13, PX],
                  [R21, R22, R23, PY],
                  [R31, R32, R33, PZ],
                  [0, 0, 0, 1]], dtype=np.float32)

    return T

def simple_ik_3d(x, y, z):
    """
    Given (x, y, z) and response (s, kappa, phi)
    by simple geometry methods.
    """

    if x == 0:
        if y > 0:
            phi = np.pi/2
        elif y < 0:
            phi = -np.pi/2
        else:
            phi = 0
    else:
        phi = np.arctan2(y, x)

    l = np.sqrt(x**2 + y**2)
    length = np.sqrt(l**2 + z**2)

    l2 = length*0.5
    cos_alpha = l/length

    r = l2/cos_alpha
    kappa = 1/r

    if np.abs(kappa) < 0.000001:
        s = length
    else:
        s = r*np.arcsin(z/r)

    return s, kappa, phi

def virtual_joints_to_skp(T0, j0, p1):
    """
    From T_base, virtual joint(j0), and point1(p1) to get s, kappa, phi.
    """
    tip_ori = p1 - j0
    l = np.linalg.norm(tip_ori)
    tip_ori = tip_ori / l

    p0 = T0[0:3, 3]

    base_ori = j0 - p0
    base_ori = base_ori / np.linalg.norm(base_ori)

    theta = np.arccos(np.dot(tip_ori, base_ori)/(np.linalg.norm(tip_ori)*np.linalg.norm(base_ori)))

    kappa = np.tan(theta/2)/l

    if kappa < 0.000001:
        s = 2*l
        kappa = 0
        phi = 0
    else:
        s = theta/kappa

        R = T0[0:3, 0:3]

        if base_ori[2] < 0:
            upsidedown_rot_mat = np.array([[-1, 0, 0],
                                        [0, 1, 0],
                                        [0, 0, -1]])
            base_ori = upsidedown_rot_mat @ base_ori.reshape(3, 1)
            tip_ori = upsidedown_rot_mat @ tip_ori.reshape(3, 1)
            R = upsidedown_rot_mat @ R

        proj_vect = R.transpose() @ tip_ori

        phi = np.arctan2(proj_vect[1], proj_vect[0])

    return s, kappa, phi
                                        

def cablelen_to_skp(l1, l2, l3, d):
    """Convert cable length to s, kappa, phi"""
    s = (l1 + l2 + l3)/3
    kappa = 2* np.sqrt(l1**2 + l2**2 + l3**2 - l1*l2 - l2*l3 - l3*l1)/(d * (l1 + l2 + l3))

    if abs(l2 - l3) < 0.01:
        if l1 <= l2:
            phi = np.pi/2
        else:
            phi = np.pi*3/2

    else:
        phi = np.arctan2(l3 + l2 - 2*l1, np.sqrt(3) * (l2 - l3))

    return s, kappa, phi

def skp_to_cablelength(s, kappa, phi, d):
    """convert s, k, p to cable length"""
    l1 = s * (1 - kappa*d*np.sin(phi));
    l2 = s * (1 + kappa*d*np.sin(np.pi/3 + phi));
    l3 = s * (1 - kappa*d*np.cos(np.pi/6 + phi));

    return l1, l2, l3


