#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
#from std_msgs.msg import Float64MultiArray
from soft_robot_vs.srv import TwoModuleJacobian

def two_module_jacobian_client(l1, l2, l3, l4, l5, l6,  d):
    rospy.wait_for_service('two_module_jacobian')
    try:
        two_module_jacobian =  rospy.ServiceProxy('two_module_jacobian', TwoModuleJacobian)
        print("got service")
        resp1 = two_module_jacobian(l1, l2, l3, l4, l5, l6, d)

        return resp1.jv
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        
def usage():
    return "%s [l1 l2 l3 d]"%sys.argv[0]

if __name__ == "__main__":
    # if len(sys.argv) == 5:
        # l1 = float(sys.argv[1])
        # l2 = float(sys.argv[2])
        # l3 = float(sys.argv[3])
        # d = float(sys.argv[4])
    l1 = 90
    l2 = 100
    l3 = 90
    l4 = 120
    l5 = 100
    l6 = 91
    d = 40
    # else:
    #     print(usage())
    #     sys.exit(1)
    print("Requesting Jacobian for (%s, %s, %s, %s, %s, %s, %s)"%(l1, l2, l3, l4, l5, l6, d))
    print(two_module_jacobian_client(l1, l2, l3, l4, l5, l6, d))
