#!/usr/bin/env python3
import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Int64, Float64MultiArray

import numpy as np
import matplotlib.pyplot as plt
from utils.continuum_misc import continuum_fk_arc, cablelen_to_skp, skp_to_cablelength
from utils.plot import plot_arc

global min_cable_len,max_cable_len,module_num,module_d
min_cable_len = 80
max_cable_len = 200
module_num = 2
module_d = 40
#* module state
cable_pos = np.array([[min_cable_len]*3]*module_num) # 3 motors per module
skp_state = np.array([[0.0]*3]*module_num)
transMatrixList = []

#* base pose
p0 = np.array([0, 0, 0])
ori0 = np.array([0, 0, -1])

T0 = np.array([[-1, 0, 0, 0],
               [0, 1, 0, 0],
               [0, 0, -1, 0],
               [0, 0, 0, 1]])

def pos_update(vel):
    global cable_pos
    m0cmd = np.array([vel[0], vel[1], vel[1]])
    m1cmd = np.array([vel[2], vel[3], vel[3]])

    cable_pos[0] = cable_pos[0] + m0cmd
    cable_pos[1] = cable_pos[1] + m1cmd

def update_transformation_matrix(module_list, skp_list, module_n):
    s, kappa, phi = skp_list[module_n]
    T = continuum_fk_arc(s, kappa, phi)
    module_list[module_n] = T

def update_cable_lenth(cbl_list, skp_list, module_n):
    s, kappa, phi = skp_list[module_n]
    l1, l2, l3 = skp_to_cablelength(s, kappa, phi, module_d)
    cbl_list[module_n] = [l1, l2, l3]

def update_skp(skp_list, cbl_list, module_n):
    l1, l2, l3 = cbl_list[module_n]
    s, kappa, phi = cablelen_to_skp(l1, l2, l3, module_d)
    skp_list[module_n] = [s, kappa, phi]

def init_T_list(T_list):
    for _ in range(module_num):
        T_list.append(T0)

def update_EE_pose(new_cable_pose):
    global cable_pos, skp_state, transMatrixList

    Tee = T0
    for seg in range(module_num):
        cable_pos[seg] = new_cable_pose[seg]
        update_skp(skp_state, cable_pos, seg)
        update_transformation_matrix(transMatrixList, skp_state, seg)

        Tee = Tee @ transMatrixList[seg]

    return Tee


def poselengths_callback(msg):
    global pose_lengths
    pose


def main(args):

    #* module prameters
    rospy.init_node('origami_sim', anonymous=True)
    pose_sub = rospy.Subscriber("origamibot/poselengths",Float64MultiArray, poselengths_callback, queue_size=1)


    
    init_T_list(transMatrixList)
    # Subscribe to pose

    new_pos = [[90, 100, 90],
               [120, 100, 91]]

    Tee = update_EE_pose(new_pos)
    print("Tee:", Tee)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    
    Tn = T0
    for seg in range(module_num):
        s, k, p = skp_state[seg]
        print(p)
        plot_arc(Tn, s, k, p, ax)
        Tn = Tn @ transMatrixList[seg]

    ax.set_box_aspect([ub - lb for lb, ub in (getattr(ax, f'get_{a}lim')() for a in 'xyz')])
    plt.show()
    # publish image topic



    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
