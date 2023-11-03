'''
Filename: hummingbird_nmpc/src/hummingbird_nmpc/state.py
Created Date: Thursday, November 2nd 2023
Author: Duy Nam Bui

Copyright (c) 2023 Duy Nam Bui
'''

from enum import Enum

class State(Enum):
    IDLE = 0
    TAKE_OFF = 1
    HOVERING = 2
    TRACKING = 3
    LANDING = 4