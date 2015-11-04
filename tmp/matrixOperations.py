#!/usr/bin/env python
import rospy
import numpy as np
from tf import transformations

def translation_from_vector(R):
  return np.array([ [R[0], R[1], R[2], R[3]],
                    [R[4], R[5], R[6], R[7]],
                    [R[8], R[9], R[10], R[11]],
                    [R[12], R[13], R[14], R[15]]]);


def euler_matrix_from_quaternion(rotQuaternion, trans):

  euler_from_quaternion = transformations.euler_from_quaternion(rotQuaternion)
  euler_matrix = transformations.euler_matrix(euler_from_quaternion[0], euler_from_quaternion[1], euler_from_quaternion[2])
  return np.array([ [euler_matrix[0,0], euler_matrix[0,1], euler_matrix[0,2], trans[0] ],
                    [euler_matrix[1,0], euler_matrix[1,1], euler_matrix[1,2], trans[1] ],
                    [euler_matrix[2,0], euler_matrix[2,1], euler_matrix[2,2], trans[2] ],
                    [                0,                 0,                 0,        1 ] ]);

#dekompozycja regulatora

#regulator proporcjonalny
