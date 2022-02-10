#!/usr/bin/python3

import numpy as np
from numpy import linalg


import cmath
import math
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi

global mat
mat=np.matrix


# ****** Coefficients ******


global d1, a2, a3, d4, d5, d6
d1 =  0.1273 #0.089159
a2 = -0.612 #-0.425
a3 = -0.5723 #-0.39225
d4 =  0.163941 #0.10915
d5 =  0.1157 #0.09465
d6 =  0.0922 #0.0823

global d, a, alph

d = mat([0.089159, 0, 0, 0.10915, 0.09465, 0.0823])
a = mat([0 ,-0.425 ,-0.39225 ,0 ,0 ,0])
alph = mat([math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0 ])


# ************************************************** FORWARD KINEMATICS


def AH( n,th,c  ):
  
  #homogeneous transformation matrix from n to c
  T_a = mat(np.identity(4), copy=False)
  T_a[0,3] = a[0,n-1] #translation about a on x axis (normal axis)
  T_d = mat(np.identity(4), copy=False)
  T_d[2,3] = d[0,n-1] #translation about d on z axis

  #rotation on z axis to align x to the common axis
  Rzt = mat([[cos(th[n-1,c]), -sin(th[n-1,c]), 0 ,0],
             [sin(th[n-1,c]),  cos(th[n-1,c]), 0, 0],
             [0,               0,              1, 0],
             [0,               0,              0, 1]],copy=False)
      
  #rotation on x axis to align z to the movement joint axis(z)
  Rxa = mat([[1, 0,                 0,                  0],
             [0, cos(alph[0,n-1]), -sin(alph[0,n-1]),   0],
             [0, sin(alph[0,n-1]),  cos(alph[0,n-1]),   0],
             [0, 0,                 0,                  1]],copy=False)

  A_i = T_d * Rzt * T_a * Rxa
        

  return A_i

#homogeneous transformation matrix from 0 to 6
def HTrans(th,c ):  
  A_1=AH( 1,th,c  )
  A_2=AH( 2,th,c  )
  A_3=AH( 3,th,c  )
  A_4=AH( 4,th,c  )
  A_5=AH( 5,th,c  )
  A_6=AH( 6,th,c  )
      
  T_06=A_1*A_2*A_3*A_4*A_5*A_6

  return T_06

# ************************************************** INVERSE KINEMATICS 

def invKine(desired_pos):# T_06
  th = mat(np.zeros((6, 8))) #each row contains all the angles for a joint for all the 8(2) possible solutions
  P_05 = (desired_pos * mat([0,0, -d6, 1]).T-mat([0,0,0,1 ]).T)
  
  # **** theta1 ****
  
  psi = atan2(P_05[2-1,0], P_05[1-1,0])
  phi = acos(d4 /sqrt(P_05[2-1,0]*P_05[2-1,0] + P_05[1-1,0]*P_05[1-1,0])) #phi= +- acos(d4/P_05xy) d4/hypotenuse
  #Has a solution in all cases except that d4 > (P_05 )xy. This happens when the origin of the 3rd frame is close to the z axis of frame 0.
  #This forms an unreachable cylinder in the spherical workspace of the UR5
  #The two solutions for theta1 correspond to the shoulder being either left or right.
  th[0, 0:4] = pi/2 + psi + phi
  th[0, 4:8] = pi/2 + psi - phi
  th = th.real
  
  # **** theta5 ****
  
  cl = [0, 4]# wrist up or down
  for i in range(0,len(cl)):
          c = cl[i]
          T_10 = linalg.inv(AH(1,th,c))
          T_16 = T_10 * desired_pos #T_06 = T_01 * T_16
          th[4, c:c+2] = + acos((T_16[2,3]-d4)/d6)
          th[4, c+2:c+4] = - acos((T_16[2,3]-d4)/d6)

  th = th.real
  
  # **** theta6 ****
  # theta6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.

  cl = [0, 2, 4, 6]
  for i in range(0,len(cl)):
          c = cl[i]
          T_10 = linalg.inv(AH(1,th,c))
          T_16 = linalg.inv( T_10 * desired_pos ) #T_61
          th[5, c:c+2] = atan2((-T_16[1,2]/sin(th[4, c])),(T_16[0,2]/sin(th[4, c])))
          
  th = th.real

  #3-Planar arm analytical approach 

  # **** theta3 ****
  cl = [0, 2, 4, 6]
  for i in range(0,len(cl)):
          c = cl[i]
          T_10 = linalg.inv(AH(1,th,c))
          T_65 = AH( 6,th,c)
          T_54 = AH( 5,th,c)
          T_14 = ( T_10 * desired_pos) * linalg.inv(T_54 * T_65)
          P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0,0,0,1]).T
          t3 = cmath.acos((linalg.norm(P_13)**2 - a2**2 - a3**2 )/(2 * a2 * a3)) # norm ?
          #two solutions: elbow either up or down
          th[2, c] = t3.real
          th[2, c+1] = -t3.real

  # **** theta2 and theta 4 ****

  cl = [0, 1, 2, 3, 4, 5, 6, 7]
  for i in range(0,len(cl)):
          c = cl[i]
          T_10 = linalg.inv(AH( 1,th,c ))
          T_65 = linalg.inv(AH( 6,th,c))
          T_54 = linalg.inv(AH( 5,th,c))
          T_14 = (T_10 * desired_pos) * T_65 * T_54
          P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0,0,0,1]).T
          
          # theta 2
          th[1, c] = -atan2(P_13[1], -P_13[0]) + asin(a3* sin(th[2,c])/linalg.norm(P_13))
          # theta 4
          T_32 = linalg.inv(AH( 3,th,c))
          T_21 = linalg.inv(AH( 2,th,c))
          T_34 = T_32 * T_21 * T_14
          th[3, c] = atan2(T_34[1,0], T_34[0,0])
  th = th.real

  return th