#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created in Aug 2019

@author: Jiaqi Yan
"""

import os
from PIL import Image
dir_path = os.path.dirname(os.path.realpath(__file__))
#intersection_fig = dir_path + "/components/imglib/intersection_states/intersection_lights.png"
parking_lot_fig = os.path.dirname(dir_path) + '/imglib/ValetParkingNew.png'

def get_background():
    return Image.open(parking_lot_fig)



