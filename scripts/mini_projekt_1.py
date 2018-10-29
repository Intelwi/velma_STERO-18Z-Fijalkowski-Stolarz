#!/usr/bin/env python
 
 
 
 # Copyright (c) 2017, Robot Control and Pattern Recognition Group,
 # Institute of Control and Computation Engineering
 # Warsaw University of Technology
 #
 # All rights reserved.
 # 
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #     * Redistributions of source code must retain the above copyright
 #       notice, this list of conditions and the following disclaimer.
 #     * Redistributions in binary form must reproduce the above copyright
 #       notice, this list of conditions and the following disclaimer in the
 #       documentation and/or other materials provided with the distribution.
 #     * Neither the name of the Warsaw University of Technology nor the
 #       names of its contributors may be used to endorse or promote products
 #       derived from this software without specific prior written permission.
 # 
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 # ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 # DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 # DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 # (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 # ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 # SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 #
 # Author: Dawid Seredynski
 #
 
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
 
import rospy
import math
import PyKDL
import copy
 
from velma_common import *
from rcprg_planner import *
#from velma_common.velma_interface import *
from control_msgs.msg import FollowJointTrajectoryResult
from rcprg_ros_utils import exitError
 
def exitError(code):
     if code == 0:
         print "OK"
         exit(0)
     print "ERROR:", code
     exit(code)

def moveBody(q_map):
     velma.moveJoint(q_map, 8.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     error = velma.waitForJoint()
     if error != 0:
         print "The action should have ended without error, but the error code is", error
         exitError(10)
 
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map, js[1], tolerance=0.1):
         exitError(10)

def moveHead(q_dest):
     velma.moveHead(q_dest, 3.0, start_time=0.5)
     if velma.waitForHead() != 0:
         exitError(4)
     rospy.sleep(0.5)
     if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(5)

def mapBuilding():
     print "To reach the goal position, some trajectory must be exetuted that contains additional, intermediate nodes"
 
     print "Moving to the right body position..."
     moveBody(q_map_right)

     print "moving head to position: right"
     q_dest = (-1.56, 0)
     moveHead(q_dest)

     print "moving head to position: right down 1"
     q_dest = (-1.56, 0.7)
     moveHead(q_dest)

     print "moving head to position: front down 1"
     q_dest = (0, 0.7)
     moveHead(q_dest)

     print "Moving to the left body position..."
     moveBody(q_map_left)

     print "moving head to position: left down 1"
     q_dest = (1.56, 0.7)
     moveHead(q_dest)

     print "moving head to position: left"
     q_dest = (1.56, 0)
     moveHead(q_dest)

     print "moving head to position: front"
     q_dest = (0, 0)
     moveHead(q_dest)

     print "Moving to the starting position..."
     moveBody(q_map_starting)

     print "moving head to position: front up"
     q_dest = (0, -0.7)
     moveHead(q_dest)

     print "Moving to the right body position..."
     moveBody(q_map_right)

     print "moving head to position: right up"
     q_dest = (-1.56, -0.7)
     moveHead(q_dest)

     print "moving head to position: right down 2"
     q_dest = (-1.56, 1.29)
     moveHead(q_dest)

     print "moving head to position: front down 2"
     q_dest = (0, 1.29)
     moveHead(q_dest)

     print "Moving to the left body position..."
     moveBody(q_map_left)

     print "moving head to position: left down 2"
     q_dest = (1.56, 1.29)
     moveHead(q_dest)

     print "moving head to position: left up"
     q_dest = (1.56, -0.7)
     moveHead(q_dest)

     print "moving head to position: front up"
     q_dest = (0, -0.7)
     moveHead(q_dest)

     print "Moving to the starting position..."
     moveBody(q_map_starting) 

     print "moving head to position: front"
     q_dest = (0, 0)
     moveHead(q_dest)

 
if __name__ == "__main__":
     # starting position
     q_map_starting = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
 
     # right position
     q_map_right = {'torso_0_joint':-1.55, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

     # left position
     q_map_left = {'torso_0_joint':1.55, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
 
     rospy.init_node('test_jimp')
 
     rospy.sleep(0.5)
 
     print "Running python interface for Velma..."
     velma = VelmaInterface()
     print "Waiting for VelmaInterface initialization..."
     if not velma.waitForInit(timeout_s=10.0):
         print "Could not initialize VelmaInterface\n"
         exitError(1)
     print "Initialization ok!\n"
 
     print "Motors must be enabled every time after the robot enters safe state."
     print "If the motors are already enabled, enabling them has no effect."
     print "Enabling motors..."
     if velma.enableMotors() != 0:
         exitError(2)
 
     rospy.sleep(0.5)
 
     diag = velma.getCoreCsDiag()
     if not diag.motorsReady():
         print "Motors must be homed and ready to use for this test."
         exitError(1)
 
     print "Switch to jnt_imp mode (no trajectory)..."
     velma.moveJointImpToCurrentPos(start_time=0.5)
     error = velma.waitForJoint()
     if error != 0:
         print "The action should have ended without error, but the error code is", error
         exitError(3)
 
     print "Moving to the starting position..."
     velma.moveJoint(q_map_starting, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     error = velma.waitForJoint()
     if error != 0:
         print "The action should have ended without error, but the error code is", error
         exitError(6)
 
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map_starting, js[1], tolerance=0.1):
         exitError(10)

     print "moving head to position: 0"
     q_dest = (0,0)
     velma.moveHead(q_dest, 1.0, start_time=0.5)
     if velma.waitForHead() != 0:
         exitError(4)
     rospy.sleep(0.5)
     if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(5)

     print "Checking if the starting configuration is as expected..."
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map_starting, js[1], tolerance=0.3):
         print "This test requires starting pose:"
         print q_map_starting
         exitError(10)
 
     rospy.sleep(1.0)
 
     # mapBuilding(); Odkrywanie otoczenia poprzez rozgladanie sie i obracanie

     # Uwzglednianie mapy
     print "waiting for Planner init..."
     p = Planner(velma.maxJointTrajLen())
     if not p.waitForInit():
         print "could not initialize PLanner"
         exitError(2)
     print "Planner init ok"
     oml = OctomapListener("/octomap_binary")
     rospy.sleep(1.0)
     octomap = oml.getOctomap(timeout_s=5.0)
     p.processWorld(octomap)

     #velma.waitForInit()
     T_B_Jar = velma.getTf("B", "beer") #roslaunch rcprg_gazebo_utils gazebo_publish_ros_tf_object.launch link_name:=beer::link frame_id:=beer
     if not velma.moveCartImpRight([T_B_Jar], [0.1], [PyKDL.Frame()], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
         exitError(8)
     """if velma.waitForEffectorRight() != 0:
         exitError(9)"""
     print "killing Michal"

     # z pliku test_cimp_imp.py :
     """print "Switch to cart_imp mode (no trajectory)..."
     if not velma.moveCartImpRightCurrentPos(start_time=0.2):
         exitError(10)
     if velma.waitForEffectorRight() != 0:
         exitError(11)
 
     rospy.sleep(0.5)
 
     diag = velma.getCoreCsDiag()
     if not diag.inStateCartImp():
         print "The core_cs should be in cart_imp state, but it is not"
         exitError(12)
 
     print "To see the tool frame add 'tf' in rviz and enable 'right_arm_tool' frame."
     print "At every state switch to cart_imp, the tool frames are reset."
     print "Also, the tool impedance parameters are reset to 1500N/m in every"\
         " direction for linear stiffness and to 150Nm/rad in every direction for angular"\
         " stiffness, i.e. (1500,1500,1500,150,150,150)."
 
     print "Moving right wrist to pose defined in world frame..."
     T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.7 , -0.3 , 1.3 ))
     if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
         exitError(13)
     if velma.waitForEffectorRight() != 0:
         exitError(14)
     rospy.sleep(0.5)
     print "Calculating difference between desiread and reached pose..."
     T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Tr"), 1.0)
     print T_B_T_diff
     if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
         exitError(15)"""

     exitError(0)
