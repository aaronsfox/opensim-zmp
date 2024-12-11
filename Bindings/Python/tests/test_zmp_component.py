"""The tests here ensure the proper functionality of modifications/additions we
make to the C++ API, via the SWIG interface (*.i) file.

"""

import os
import unittest
from math import isnan

import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

class TestSwigAddtlInterface(unittest.TestCase):
    
    def test_ZeroMomentPointGroundReactions(self):
        
        # Test creating and editing ZMP ground reactions component
        zmpGroundReactions = osim.ZeroMomentPointGroundReactions()
        zmpGroundReactions.setName('zmpGroundReactions')
        zmpGroundReactions.setFreeJointName('ground_pelvis')
        zmpGroundReactions.setForceThreshold(50.0)
        
        # Print an output from the component to test
        print(f'ZMP Ground Reactions Object Name: {zmpGroundReactions.getName()}')
        
        # Test creating a ZMP contact body
        # TODO: allocate after finalising class names...