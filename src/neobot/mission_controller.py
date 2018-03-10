#! /usr/bin/env python

from __future__ import division
from transitions.extensions import HierarchicalMachine as Machine
#from transitions.extensions.nesting import NestedState as nState
#from transitions import Machine as Machine
# see https://github.com/tyarkoni/transitions
from tf import transformations as t
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi, sin, cos
import copy
import time

class CliffHandler(Machine):
    
    def __init__(self):
        self.states = [{'name': 'angle_finding', 'on_enter': self.save_start_pose,
                'on_exit': self.save_end_pose},
                'aligning', 'backing', 'spinning', 'done']
        Machine.__init__(self, states=self.states, initial='angle_finding')
        self.add_transition('at_cliff', 'angle_finding', 'aligning')
        self.add_transition('at_goal', 'aligning', 'backing')
        self.add_transition('at_goal', 'backing', 'spinning')
        self.add_transition('at_goal', 'spinning', 'done')
        self.add_transition('at_goal', 'angle_finding', 'backing')
        self.add_transition('at_cliff', 'aligning', 'backing')
        
    def save_start_pose(self):
        print('Executing save_start_pose')
    
    def save_end_pose(self):
        print('Executing save_end_pose')
  
class Seeker(Machine):
            
    def __init__(self, num_edges):
        self.max_edges = num_edges  # stops seeking after finding this many edges
        self.reset_edge_count()
        self.ch = CliffHandler()
        self.states = ['idle', 'edge_finding', 
            {'name': 'clearing', 'children': self.ch, 'remap': {'done': 'cleared'}}, 
            {'name': 'cleared', 'on_enter': 'check_if_done'}, 'final_move',
            'mission_complete']
        Machine.__init__(self, states = self.states, initial = 'idle')
        self.add_transition('start', 'idle', 'edge_finding',
                            before = ['reset_edge_count'])
        self.add_transition('at_cliff', 'edge_finding', 'clearing',
                            before = ['increment_edge_count'],
                            after = 'to_clearing_angle_finding')
        self.add_transition('at_goal', 'final_move', 'mission_complete')
        
    def get_goal_pose(self, frame='odom'):
        pass
        # TODO - should this be up a level or two? Does ch need its own?
    
    def reset_edge_count(self):
        self.edges = 0
        
    def increment_edge_count(self):
        self.edges += 1
    
    def check_if_done(self):
        if self.edges < self.max_edges:
            self.to_edge_finding()
        else:
            #self.to_mission_complete()
            self.to_final_move()
            
    def make_final_move(self):
        print('Executing final move')
        
    
class MissionController:
    """Sets goal poses for the bot to seek.
    
    Start development by only including the lower level (nested) FSM that
    handles cliff-clearing.
    """
    
    def __init__(self, num_edges=2):
        self.num_edges = num_edges  # number of table edges to seek
        self.seeker = Seeker(num_edges)    
        self.start_angle_pose = None
        self.end_angle_pose = None
        self.edge_poses = []
    
    def point_to_numpy(self, msg, hom=False):
        if hom:
            return np.array([msg.x, msg.y, msg.z, 1])
        else:
            return np.array([msg.x, msg.y, msg.z])
            
    def quat_to_numpy(self, msg):
        return np.array([msg.x, msg.y, msg.z, msg.w])
        
    def straight_translation(self, pose, dist):
        """
        Args:
            pose: geometry_msgs Pose() representing starting pose
            dist: signed distance to translate on current heading
        """
        pos_np = self.point_to_numpy(pose.position)
        orn_np = self.quat_to_numpy(pose.orientation)
        heading = t.euler_from_quaternion(orn_np, axes='sxyz')[2]  # radians
        translation = np.array([dist * cos(heading), dist * sin(heading), 0.0])
        new_pose = Pose()
        new_position_np = np.add(pos_np, translation)
        new_pose.position = Point(*new_position_np)
        new_pose.orientation = pose.orientation
        return new_pose
        
    def pure_spin_move(self, pose, ang):
        """
        Args:
            pose: geometry_msgs Pose9) representing starting pose
            ang: signed radian angle to spin about current position
        """
        new_pose = Pose()
        orn = pose.orientation
        q_spin = t.quaternion_from_euler(0, 0, ang, axes='sxyz')
        q_new_np = t.quaternion_multiply(q_spin, self.quat_to_numpy(orn))
        orn_new = Quaternion(*q_new_np)
        new_pose.position = pose.position
        new_pose.orientation = orn_new
        return new_pose
    
    def get_goal(self, pose, turn_dir):  # pose is a geometry_msgs Pose # turn_dir is int
        """Specifies the new goal pose.
        
        Args:
            pose: bot's current pose
            turn_dir: direction to rotate recovering from a cliff (sensor based)
                +1 is CCW; -1 is CW; 0 is don't rotate (skip angle_finding)
        """
        position = pose.position
        orn = pose.orientation
        goal_pose = Pose()  # empty geometry_msgs Pose
        
        if self.seeker.state == 'edge_finding':
            #~ d = 3.0  # set distant straightahead goal
            goal_pose = self.straight_translation(pose, 3.0)
            
        elif self.seeker.state == 'clearing_angle_finding':
            # set goal 2*pi/3 rad spin; direction depends on cliff_status
            # need to interpret cliff_status
            # for temp testing cliff_status = +1 --> recover ccw; -1 --> recover cw
            
            # save initial pose here (start of angle); eliminate in CH
            self.start_angle_pose = copy.deepcopy(pose)            
            # also save here for table dimension estimating; possibly redundant
            self.edge_poses.append(pose)
            ang = 15*pi/16 * turn_dir
            goal_pose = self.pure_spin_move(pose, ang)

        elif self.seeker.state == 'clearing_aligning':
            # save end pose for alignment; redundant?
            self.end_angle_pose = pose  # assumes current pose resulted from angle_finding
            # now want to rotate from end_angle_pose halfway to start_angle_pose
            # slerp gives the desired *goal orientation* midway between end and start args
            q_end = self.end_angle_pose.orientation
            q_start = self.start_angle_pose.orientation
            q_end_np = self.quat_to_numpy(q_end)
            q_start_np = self.quat_to_numpy(q_start)
            q_mid_np = t.quaternion_slerp(q_end_np, q_start_np, 0.5)
            yaw_centering = t.euler_from_quaternion(q_mid_np, axes='sxyz')[2]
            orn_goal = Quaternion(*q_mid_np)
            goal_pose.position = position
            goal_pose.orientation = orn_goal
            
        elif self.seeker.state == 'clearing_backing':
            # set goal 0.1 m straight reverse
            goal_pose = self.straight_translation(pose, -0.2)
            
        elif self.seeker.state == 'clearing_spinning':
            # set goal pi rad spin
            ang = pi
            goal_pose = self.pure_spin_move(pose, ang)
            
        elif self.seeker.state == 'cleared':
            # do we need this???
            print('========= Reached "cleared" state =========')
            
        elif self.seeker.state == 'final_move':
            # set final move 'x' m straight forward depending on table dimension
            # for now set goal 0.2 m straight forward
            goal_pose = self.straight_translation(pose, 0.2)
            
        elif self.seeker.state == 'mission_complete':
            # stay where you are
            goal_pose = pose
            
            print('======== Completed mission! =============')
        else:
            print('======== Unexpected state in "mission publish" ===========')  # TODO
        
        return goal_pose
        
