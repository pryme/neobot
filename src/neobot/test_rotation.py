#! /usr/bin/env python

# test spin move calcs using transformations.py

from tf import transformations as t
import numpy as np
from math import pi, sin, cos
import sys

def normalize_angle(a):
        """Normalizes an angle to range (-pi, pi].
        
        Args:
            a: An angle in radians.
            
        Returns:
            Normalized angle in radians in range (-pi, pi].
        """
        a = a % (2 * pi)
        if a > pi:
            a -= 2 * pi
        return(a)

start_ang = float(sys.argv[1]) / 180.0 * pi  # enter start heading angle (deg) (+ --> ccw, 0 --> East)
spin_ang = float(sys.argv[2]) / 180.0 * pi  # enter angle (deg) to spin thru (+ --> ccw)

end_ang = normalize_angle(start_ang + spin_ang)

q_start = t.quaternion_from_euler(0, 0, start_ang, axes='sxyz')
q_spin = t.quaternion_from_euler(0, 0, spin_ang, axes='sxyz')
#q_end = t.quaternion_multiply(q_start, q_spin)
q_end = t.quaternion_multiply(q_spin, q_start)
yaw_start = normalize_angle(start_ang)
yaw_end = t.euler_from_quaternion(q_end, axes='sxyz')[2]

print('==============')
print('yaw diff:\t {0:.3f}'.format(normalize_angle(yaw_end - end_ang)))
print('q_start:\t[' + ', '.join('{0:.3f}'.format(x) for x in q_start) + ']')
print('q_end:\t\t[' + ', '.join('{0:.3f}'.format(x) for x in q_end) + ']')
print(' ')

# now want to rotate from q_end to midpoint between q_end and q_start
# slerp gives the desired *goal orientation* midway between end and start
s = np.dot(q_end, q_start)
q_mid = t.quaternion_slerp(q_end, q_start, 0.5)
yaw_centering = t.euler_from_quaternion(q_mid, axes='sxyz')[2]
yaw_avg = normalize_angle(yaw_end - (yaw_end - yaw_start)/2.0)
print('s:\t\t{0:.3f}'.format(s))
print('q_mid:\t\t[' + ', '.join('{0:.3f}'.format(x) for x in q_mid) + ']')
print('yaws (end, mid, start, avg):\t{0:.3f}\t{1:.3f}\t{2:.3f}\t\t{3:.3f}'.format(yaw_end, yaw_centering, yaw_start, yaw_avg))

