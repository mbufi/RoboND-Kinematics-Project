#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya, Richie Muniak

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
import random
from mpmath import *
from sympy import *
import numpy as np


#Setting up global Parameters
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
rollsymbol = symbols('roll')
pitchsymbol = symbols('pitch')
yawsymbol = symbols('yaw')

T0_1 = 0
T1_2 = 0
T2_3 = 0
T3_4 = 0
T4_5 = 0
T5_6 = 0
T6_7 = 0
T_Final = 0
R_corr = 0
R_EE = 0

#Setting up global DH Table
s = {alpha0:      0, a0:       0, d1:  0.75,
     alpha1:  -np.pi/2, a1:    0.35, d2:     0, q2: q2-np.pi/2,
     alpha2:      0, a2:    1.25, d3:     0,
     alpha3:  -np.pi/2, a3:  -0.054, d4:  1.50,
     alpha4:   np.pi/2, a4:       0, d5:     0,
     alpha5:  -np.pi/2, a5:       0, d6:     0,
     alpha6:      0, a6:       0, d7: 0.303, q7: 0}

#To calculate for end effector
R_z = Matrix([[cos(yawsymbol), -sin(yawsymbol), 0],
              [sin(yawsymbol), cos(yawsymbol), 0],
              [0, 0, 1]])

R_y = Matrix([[cos(pitchsymbol), 0, sin(pitchsymbol)],
              [0, 1, 0],
              [-sin(pitchsymbol), 0, cos(pitchsymbol)]])

R_x = Matrix([[1, 0, 0],
              [0, cos(rollsymbol), -sin(rollsymbol)],
              [0, sin(rollsymbol), cos(rollsymbol)]])

#Run the function before the IK to save time. Thanks to forums for this...
def initalization():
    global T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_7, T_Final, R_EE, R_x, R_y, R_z,R_corr

    #The forward kinematic transforms.  Ripped right from the lectures.
    T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                   0,                   0,            0,               1]])
    T0_1 = T0_1.subs(s)

    T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
               [                   0,                   0,            0,               1]])
    T1_2 = T1_2.subs(s)

    T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
               [                   0,                   0,            0,               1]])
    T2_3 = T2_3.subs(s)

    T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
               [                   0,                   0,            0,               1]])
    T3_4 = T3_4.subs(s)

    T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
               [                   0,                   0,            0,               1]])
    T4_5 = T4_5.subs(s)

    T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
               [                   0,                   0,            0,               1]])
    T5_6 = T5_6.subs(s)

    T6_7 = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
               [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
               [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
               [                   0,                   0,            0,               1]])
    T6_7 = T6_7.subs(s)
    
    #The link from the begginging all the way to the end
    T0_7 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7
    
    #How to fix for gribber, ripped right from video
    R_zz = Matrix([[ cos(np.pi), -sin(np.pi), 0,0],
              [ sin(np.pi),  cos(np.pi), 0,0],
              [          0,           0, 1,0],
              [          0,           0, 0,1]])
    R_yy = Matrix([[  cos(-np.pi/2), 0, sin(-np.pi/2),0],
              [              0, 1,             0,0],
              [ -sin(-np.pi/2), 0, cos(-np.pi/2),0],
              [              0, 0,             0,1]])
    R_corr = R_zz * R_yy


    #The final total homogenous transformation in all its glory.
    T_Final = T0_7 * R_corr
    
#The Ros srv function.
def handle_calculate_IK(req):
    global R_EE,R_corr, T_Final
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))

    if len(req.poses) < 1:
        print "No valid poses received"
        return -1 #It shouldn't be this, but I don't know what it should be.
    else:

        # Initialize service response
        joint_trajectory_list = []
        # 0 out our thetas for fun and initialization
        theta1 = theta2 = theta3 = theta4 = theta5 = theta6 = 0
        for x in xrange(0, len(req.poses)):
            # IK code starts here
        
            joint_trajectory_point = JointTrajectoryPoint()
        
            # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            #final rotation of end effector.
            R_EE = (R_x * R_y * R_z).evalf(subs={rollsymbol:roll,pitchsymbol:pitch,yawsymbol:yaw})
            
            # find the wrist from the rotation matrix
            wx = (px - (d6 + d7) * R_EE[0,0]).subs(s)
            wy = (py - (d6 + d7) * R_EE[1,0]).subs(s)
            wz = (pz - (d6 + d7) * R_EE[2,0]).subs(s)

            theta1 = atan2(wy, wx)
            
            # Need to account for 0.054
            internal_angle = atan2(wz - 1.94645, wx)
            wx = wx - 0.054 * sin(internal_angle)
            wz = wz + 0.054 * cos(internal_angle)

            # Finding the distance from the origin to the newly slightly moved wrist center
            wxdist = sqrt(wy*wy+wx*wx)

            # grab the lengths for the segments to calculate theta 2/3 from DH table
            l1 = s[a2]
            l2 = s[d4]

            # Moving the second joint to the origin to make the math cleaner later.
            wxdist = wxdist - s[a1]
            wzdist = wz - s[d1]

            # Cosine law
            D=(wxdist*wxdist + wzdist*wzdist - l1*l1-l2*l2)/(2*l1*l2)
            
            # Clip d from going above 1
            if (D>1):
                D=1

            theta3 = atan2(-sqrt(1-D*D),D)
            
            # equation Pulled from book
            sine = ((l1 + l2 * cos(theta3)) * wzdist - l2 * sin(theta3) * wxdist) / (wxdist * wxdist + wzdist * wzdist)
            cosine = ((l1 + l2 * cos(theta3)) * wxdist + l2 * sin(theta3) * wzdist) / (wxdist * wxdist + wzdist * wzdist)
            theta2 = atan2(sine, cosine)
            
            # Theta3 needs to be translated by 90 degrees
            theta3 = -1*(theta3+pi/2)
            theta2 = pi/2-theta2

            # R0_3 is the rotation matrix up the wrist, we can now get this because we have theta1-theta3
            R0_3 = (T0_1 * T1_2 * T2_3).evalf(subs={q1:theta1,q2:theta2,q3:theta3})[0:3,0:3]
            
            # We know up to the wrist, and we know after the wrist, this gives us the last three rotations
            # in a rotation matrix
            R3_6 = (R0_3)**-1 * R_EE[0:3,0:3]

            # Formula for taking the rotation to euler
            (theta4, theta5, theta6) = tf.transformations.euler_from_matrix(np.array(R3_6[0:3,0:3]).astype(np.float64),"ryzx")

            
            #Due to 0 case, translate theta 5 by 90 degrees
            theta5 = (theta5 - np.pi/2)
            
            # Sometimes theta 5 likes to go crazy and bump into itself.  This stops that.
            if (theta5 > 2):
                theta5 = 2
            if (theta5 < -2):
                theta5 = -2
            #Due to 0 case, translate theta 6 by 90 degrees
            theta6 = theta6 - np.pi/2

            # Adding our joints to the list to send back.
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

            print("Poses")
            print("x", px, "y", py, "z", pz)
            print("roll", roll, "pitch", pitch, "yaw", yaw)

            T_Final = T_Final.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
            print("Forward Kinematics to end:")
            print(T_Final)


        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    initalization()
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()

