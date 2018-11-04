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
from threading import Thread
 
from velma_common import *
from rcprg_planner import *
#from velma_common.velma_interface import *
from control_msgs.msg import FollowJointTrajectoryResult
from rcprg_ros_utils import MarkerPublisher, exitError

from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import tf_conversions.posemath as pm
 
class MarkerPublisherThread:
    def threaded_function(self, obj):
        pub = MarkerPublisher("attached_objects")
        while not self.stop_thread:
            pub.publishSinglePointMarker(PyKDL.Vector(), 1, r=1, g=0, b=0, a=1, namespace='default', frame_id=obj.link_name, m_type=Marker.CYLINDER, scale=Vector3(0.02, 0.02, 1.0), T=pm.fromMsg(obj.object.primitive_poses[0]))
            try:
                rospy.sleep(0.1)
            except:
                break
 
        try:
            pub.eraseMarkers(0, 10, namespace='default')
            rospy.sleep(0.5)
        except:
            pass
 
    def __init__(self, obj):
        self.thread = Thread(target = self.threaded_function, args = (obj, ))

    def start(self):
        self.stop_thread = False
        self.thread.start()

    def stop(self):
        self.stop_thread = True
        self.thread.join()

# wspolrzedne puszki
x_p=0
y_p=0
z_p=0
theta1=0

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
def initVelma():
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

     print "reset left"
     velma.resetHandLeft()
     if velma.waitForHandLeft() != 0:
         exitError(2)
     rospy.sleep(0.5)
     if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), [0,0,0,0]):
         exitError(3)
 
     print "reset right"
     velma.resetHandRight()
     if velma.waitForHandRight() != 0:
         exitError(4)
     rospy.sleep(0.5)
     if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), [0,0,0,0]):
         exitError(5)
 
     rospy.sleep(1.0)

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

def findObject(object): #trzeba poprawic by zawsze sie obracal
     #velma.waitForInit()
     T_B_Jar = velma.getTf("B", object) #odebranie pozycji i orientacji obiektu
   
     z = T_B_Jar.p[2]#z
     y = T_B_Jar.p[1]#y
     x = T_B_Jar.p[0]#x

     theta = math.atan2(y,x)
     if theta>1.55:
	theta = 1.55
     if theta<-1.55:
	theta = -1.55

     return x,y,z,theta


def findPlanExecute():
     [x_p,y_p,z_p,theta] = findObject("beer") #znalezienie piwa
     q_map_change['torso_0_joint'] = theta
     handsUp(); #edit joints
     return x_p, y_p, z_p

def handsUp():
     q_map_change['right_arm_0_joint'] = 1
     q_map_change['right_arm_1_joint'] = -1.2
     q_map_change['right_arm_3_joint'] = 2

def toCart():
     print "Switch to cart_imp mode (no trajectory)..."
     if not velma.moveCartImpRightCurrentPos(start_time=0.2):
         exitError(8)
     if velma.waitForEffectorRight() != 0:
         exitError(9)
     rospy.sleep(0.5)

def toJnp():
     print "Switch to jnt_imp mode (no trajectory)..."
     velma.moveJointImpToCurrentPos(start_time=0.5)
     error = velma.waitForJoint()
     if error != 0:
         print "The action should have ended without error, but the error code is", error
         exitError(3)

def gripper_action(gripper,action):
     if gripper == "left":	
     	if action == "grab":
     		dest_q = [80.0/180.0*math.pi,80.0/180.0*math.pi,80.0/180.0*math.pi,0]
     		print "move left:", dest_q
     		velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
     		
     	elif action == "drop":
     		dest_q = [0,0,0,0]
     		print "move left:", dest_q
     		velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
	else:
		return
     	rospy.sleep(1)
     elif gripper == "right":	
     	if action == "grab":
     		dest_q = [80.0/180.0*math.pi,80.0/180.0*math.pi,80.0/180.0*math.pi,0]
     		print "move right:", dest_q
     		velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
     		
     	elif action == "drop":
     		dest_q = [0,0,0,0]
     		print "move right:", dest_q
     		velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
	else:
		return
     	rospy.sleep(1)
     else:
     	return

def writeJointStateToQMAP(q_map_change,actual_joints):
     q_map_change['torso_0_joint'] = actual_joints[1]['torso_0_joint']
     q_map_change['right_arm_0_joint'] = actual_joints[1]['right_arm_0_joint']
     q_map_change['right_arm_1_joint'] = actual_joints[1]['right_arm_1_joint']
     q_map_change['right_arm_2_joint'] = actual_joints[1]['right_arm_2_joint']
     q_map_change['right_arm_3_joint'] = actual_joints[1]['right_arm_3_joint']
     q_map_change['right_arm_4_joint'] = actual_joints[1]['right_arm_4_joint']
     q_map_change['right_arm_5_joint'] = actual_joints[1]['right_arm_5_joint']
     q_map_change['right_arm_6_joint'] = actual_joints[1]['right_arm_6_joint']
     q_map_change['left_arm_0_joint'] = actual_joints[1]['left_arm_0_joint']
     q_map_change['left_arm_1_joint'] = actual_joints[1]['left_arm_1_joint']
     q_map_change['left_arm_2_joint'] = actual_joints[1]['left_arm_2_joint']
     q_map_change['left_arm_3_joint'] = actual_joints[1]['left_arm_3_joint']
     q_map_change['left_arm_4_joint'] = actual_joints[1]['left_arm_4_joint']
     q_map_change['left_arm_5_joint'] = actual_joints[1]['left_arm_5_joint']
     q_map_change['left_arm_6_joint'] = actual_joints[1]['left_arm_6_joint']
     return q_map_change

def moveCart(B_T):
     print "Zaczynam ruch nadgarstka"
     if not velma.moveCartImpRight([B_T], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
         exitError(16)
     if velma.waitForEffectorRight() != 0:
         exitError(17)
     rospy.sleep(0.5)
     print "calculating difference between desiread and reached pose..."
     T_B_T_diff = PyKDL.diff(B_T, velma.getTf("B", "Tr"), 1.0)
     print T_B_T_diff
     if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
         exitError(10)
#-----------------------------------------------------MAIN---------------------------------------------------------------#
 
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

      # changing dictionary
     q_map_change = {'torso_0_joint':0,
         'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8, 'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85,
         'right_arm_4_joint':0, 'right_arm_5_joint':-0.5, 'right_arm_6_joint':0,
         'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25, 'left_arm_3_joint':-0.85,
         'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
 
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
 
     print "waiting for Planner init..."
     p = Planner(velma.maxJointTrajLen())
     if not p.waitForInit():
         print "could not initialize PLanner"
         exitError(2)
     print "Planner init ok"

     # define a function for frequently used routine in this test
     def planAndExecute(q_dest):
         print "Planning motion to the goal position using set of all joints..."
         print "Moving to valid position, using planned trajectory."
         goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
         for i in range(20):
             rospy.sleep(0.5)
             js = velma.getLastJointState()
             print "Planning (try", i, ")..."
             traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
             if traj == None:
                 continue
             print "Executing trajectory..."
             if not velma.moveJointTraj(traj, start_time=0.5):
                 exitError(5)
             if velma.waitForJoint() == 0:
                 break
             else:
                 print "The trajectory could not be completed, retrying..."
                 continue
         rospy.sleep(0.5)
         js = velma.getLastJointState()
         if not isConfigurationClose(q_dest, js[1]):
             exitError(6)


     diag = velma.getCoreCsDiag()
     if not diag.motorsReady():
         print "Motors must be homed and ready to use for this test."
         exitError(1)
 
     toJnp(); #ruch w przestrzeni satwow
 
     #initVelma(); #-----------------------------------------------------------------!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!???????????????@@@@@@@@@@@@@@@@@
 
     #mapBuilding(); #Odkrywanie otoczenia poprzez rozgladanie sie i obracanie


     # Uwzglednianie mapy
     oml = OctomapListener("/octomap_binary")
     rospy.sleep(1.0)
     octomap = oml.getOctomap(timeout_s=5.0)
     p.processWorld(octomap)
     
     gripper_action("right","grab"); #chowamy paluszki
     gripper_action("left","grab"); #chowamy paluszki
     (x,y,z)=findPlanExecute(); #znajdowanie puszki i obliczenie kata
     planAndExecute(q_map_change); #obracanie torsu do puszki i podnoszenie rak
     print "ROTATION OK"
     print "RECE W GORZE"
    
     gripper_action("right","drop"); #wystawiamy paluszki
     toCart(); #przejscie do trybu cart_imp
     x_p=x
     y_p=y
     z_p=z
     [x,y,z,theta] = findObject("beer") 
     rot = PyKDL.Rotation.RPY(0, 0, theta)

     x_new = x_p-0.25 #przysuniecie chwytaka do puszki
     y_new = ((0-y_p)/(0-x_p))*x_new #rownanie prostej
     B_T = PyKDL.Frame(rot, PyKDL.Vector(x_new,y_new, z_p+0.3)) #tworzenie macierzy jednorodnej do ustawienia chwytaka
     moveCart(B_T); #ruch chwytakiem
     actual_joints = velma.getLastJointState() #zapamietanie czasu i aktualnej pozycji stawow
     q_map_change = writeJointStateToQMAP(q_map_change,actual_joints) #wyluskanie pozycji stawow

     x_new = x_p-0.35 #ustawienie chwytaka w odstepie od puszki
     y_new = ((0-y_p)/(0-x_p))*x_new #rownanie prostej
     B_T = PyKDL.Frame(rot, PyKDL.Vector(x_new,y_new, z_p+0.1)) #tworzenie macierzy jednorodnej do ustawienia chwytaka
     moveCart(B_T); #ruch chwytakiem

     x_new = x_p-0.25 #przysuniecie chwytaka do puszki
     y_new = ((0-y_p)/(0-x_p))*x_new #rownanie prostej
     B_T = PyKDL.Frame(rot, PyKDL.Vector(x_new,y_new, z_p+0.1)) #tworzenie macierzy jednorodnej do ustawienia chwytaka
     moveCart(B_T); #ruch chwytakiem
     
     toJnp(); #ruch w przestrzeni stawow
     gripper_action("right","grab"); #zlapanie piwka

     # ruch z planowaniem i chwyconym piwem  
     print "Creating a virtual object attached to gripper..."
     # for more details refer to ROS docs for moveit_msgs/AttachedCollisionObject
     object1 = AttachedCollisionObject()
     object1.link_name = "right_HandGripLink"
     object1.object.header.frame_id = "right_HandGripLink"
     object1.object.id = "object1"
     object1_prim = SolidPrimitive()
     object1_prim.type = SolidPrimitive.CYLINDER
     object1_prim.dimensions=[None, None]    # set initial size of the list to 2
     object1_prim.dimensions[SolidPrimitive.CYLINDER_HEIGHT] = 1.0
     object1_prim.dimensions[SolidPrimitive.CYLINDER_RADIUS] = 0.02
     object1_pose = pm.toMsg(PyKDL.Frame(PyKDL.Rotation.RotY(math.pi/2)))
     object1.object.primitives.append(object1_prim)
     object1.object.primitive_poses.append(object1_pose)
     object1.object.operation = CollisionObject.ADD
     object1.touch_links = ['right_HandPalmLink',
         'right_HandFingerOneKnuckleOneLink',
         'right_HandFingerOneKnuckleTwoLink',
         'right_HandFingerOneKnuckleThreeLink',
         'right_HandFingerTwoKnuckleOneLink',
         'right_HandFingerTwoKnuckleTwoLink',
         'right_HandFingerTwoKnuckleThreeLink',
         'right_HandFingerThreeKnuckleTwoLink',
         'right_HandFingerThreeKnuckleThreeLink']
 
     print "Publishing the attached object marker on topic /attached_objects"
     pub = MarkerPublisherThread(object1)
     pub.start()

     [x,y,z,theta] = findObject("cafe_table") #znalezenie stolika do kawy
     print "Kat do stolika od kawy "
     print theta
     
     planAndExecute(q_map_change); #podniesienie piwa
     q_map_change['torso_0_joint'] = theta
     planAndExecute(q_map_change); #ruch do stolika do kawy
     gripper_action("right","drop"); #puszczenie piwka

     pub.stop()

     gripper_action("right","grab"); #chowamy paluszki
     q_map1 = q_map_starting
     q_map1['torso_0_joint'] = theta
     planAndExecute(q_map1)

     exitError(0)
