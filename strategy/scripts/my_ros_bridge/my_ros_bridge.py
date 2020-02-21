#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
import numpy as np
import time
import message_filters
import actionlib

class Robot(object):
  
  def __init__(self, sim = False):
    if not sim :
      pass
    else:
      pass