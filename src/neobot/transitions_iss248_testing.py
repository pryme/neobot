#! /usr/bin/env python

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState as State
from transitions.extensions.nesting import NestedTransition as Transition

class Nested(Machine):
    def print_msg(self):
        print("Nested")
        #raises AttributeError exception**
        self.top.print_msg()
        
    def __init__(self, top):
        self.top = top
        self.states = ['n1', {'name':'n2', 'on_enter': self.print_msg}]
        Machine.__init__(self, states=self.states, initial='n1')
        self.add_transition(trigger='goto_n2',
                            source='*',
                            dest='n2')

class Top(Machine):
    def set_var(self, var):
        self.var = var
    def print_msg(self):
        print("Top: " + self.var)
    def __init__(self):
        self.nested = Nested(self)

        self.states = [  't1',
                        {'name': 't2',
                        'children': self.nested}]
        Machine.__init__(self, states=self.states, initial='t1')
        self.add_transition(trigger='print_top',
                            source='*',
                            dest='=',
                            after= self.print_msg)
        self.add_transition(trigger='goto_t2',
                            source='*',
                            dest='t2_n1')
