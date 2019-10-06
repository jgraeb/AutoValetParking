#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created in Aug 2019

@author: Jiaqi Yan (jiaqi@caltech.edu)

"""

import threading
import sys

# set a new thread to read user's input
class Read_user_command(threading.Thread):
    index = 0

    def __init__(self):
        threading.Thread.__init__(self)
        
    def run(self):
        while (1):
            input_key = str(sys.stdin.readline()).strip("\n")
            if input_key =='s':   # the user input 's' if he wants to choose/change the vehicle to monitor 
                self.index = 1
                

