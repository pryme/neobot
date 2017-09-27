#! /usr/bin/env python

# intended for dev testing of MissionController class

from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Int16
import numpy as np
from tf import transformations as t
from math import pi

from neobot import mission_controller

NUM_EDGES = 2
mc = mission_controller.MissionController(NUM_EDGES)
c_status = Int16(1)  # sim cliff_status msg

goal_pose = Pose()
cur_pose = Pose()  # sim pose arg
cur_pose.position = Point(0.0, 0.0, 0.0)
cur_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

def print_pose_msg(name, pose):
    p = pose.position
    q = pose.orientation
    print(name),
    #print p,
    #print q
    print('position (xyz): [{:.3f}, {:.3f}, {:.3f}]'.format(p.x, p.y, p.z)),
    print('\tquaternion (xyzw): [{:.3f}, {:.3f}, {:.3f}, {:.3f}]'.format(q.x, q.y, q.z, q.w))
    
def print_summary(pose1, pose2):
    print('\nNew state: {}\t{}'.format(mc.seeker.state, 'New goal:'))
    print_pose_msg('Goal', pose1)
    yaw, yaw_deg = yaw_from_msg_quat(goal_pose.orientation)
    print('Goal yaw: {:.3f};\t(deg): {}'.format(yaw, yaw_deg))
    print_pose_msg('Stop', pose2)
    stop_yaw, stop_yaw_deg = yaw_from_msg_quat(cur_pose.orientation)
    print('Stop yaw: {:.3f};\t(deg): {}'.format(stop_yaw, stop_yaw_deg))
  
def yaw_from_msg_quat(msg_quat):
    q_np = np.array([msg_quat.x, msg_quat.y, msg_quat.z, msg_quat.w])
    yaw = t.euler_from_quaternion(q_np)[2]
    yaw_deg = yaw * 180.0 / pi
    return (yaw, yaw_deg)

mc.seeker.start() # start edge_finding
goal_pose = mc.get_goal(cur_pose, c_status)
cur_pose.position = Point(1.0, 0.0, 0.0)  # fake first cliff at 1.0m fwd
print_summary(goal_pose, cur_pose)

while mc.seeker.state == 'edge_finding':
    mc.seeker.at_cliff()  # start angle_finding
    goal_pose = mc.get_goal(cur_pose, c_status)
    q_stop_np = t.quaternion_from_euler(0, 0, c_status.data*(pi/3), axes='sxyz')  # fake 60 deg turn
    cur_pose.orientation = Quaternion(*q_stop_np)    
    print_summary(goal_pose, cur_pose)

    mc.seeker.at_cliff()  # start aligning
    goal_pose = mc.get_goal(cur_pose, c_status)
    cur_pose = goal_pose  # assume hit goal exactly
    print_summary(goal_pose, cur_pose)

    mc.seeker.at_goal() # start backing
    goal_pose = mc.get_goal(cur_pose, c_status)
    cur_pose = goal_pose  # assume hit goal exactly
    print_summary(goal_pose, cur_pose)

    mc.seeker.at_goal()  # start spinnin
    goal_pose = mc.get_goal(cur_pose, c_status)
    cur_pose = goal_pose  # assume hit goal exactly    
    print_summary(goal_pose, cur_pose)

    mc.seeker.at_goal()  # start next cycle
    goal_pose = mc.get_goal(cur_pose, c_status)
    cur_pose = goal_pose  # assume hit goal exactly
    print_summary(goal_pose, cur_pose)
