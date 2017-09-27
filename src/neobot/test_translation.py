#! /usr/bin/env python

# test of certain transforms using transformations.py

from tf import transformations as t
import numpy as np
from math import pi, sin, cos
import sys
from geometry_msgs.msg import Pose, Point

ang = float(sys.argv[1])

#~ dir1 = (1, 0, 0)  # direction vector
#~ T = t.translation_matrix(dir1)
#~ print('T: ')
#~ print(T)
#~ print('\n')

point = [0, 0, 0, 1]
#print np.dot(T, point)  # translates the point

Rz = t.rotation_matrix(pi/2, (0, 0, 1))
#~ print('Rz: ')
#~ print(Rz)
#~ print('\n')
#~ print('Rotated:')
#~ print np.dot(Rz, point)

#~ M = t.concatenate_matrices(T, Rz)
#~ combined = np.dot(M, point)
#~ print(combined)

start = np.array((-1.0, 1.0, 0.0))
print('start:\t[' + ', '.join('{0:.3f}'.format(x) for x in start)) + ']'
angle = ang/180.0 * pi
q1 = t.quaternion_from_euler(0, 0, angle, axes='sxyz')
print('angle:\t' + '{0:.3f}'.format(angle) + '\tq1:\t[' + ', '.join('{0:.3f}'.format(x) for x in q1)) + ']'
dist = -1.0  # translation distance
yaw = t.euler_from_quaternion(q1, axes='sxyz')[2]
#goal = start + np.array((dist * cos(yaw), dist * sin(yaw), 0.0))
goal = np.add(start, np.array((dist * cos(yaw), dist * sin(yaw), 0.0)))
print('yaw:\t' + '{0:.3f}'.format(yaw) + '\tgoal:\t[' + ', '.join('{0:.3f}'.format(x) for x in goal)) + ']'

pose = Pose()
pos = pose.position
pos.x = 1.0
pos.y = 2.0
pos.z = 3.0
print('position: {}'.format(pos))
pos_np = np.array([pos.x, pos.y, pos.z])
print('new_position: {}'.format(start + pos_np))
print(' ')
test_Point = Point(3.0, 2.0, 1.0)
print(test_Point)


