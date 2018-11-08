#!/usr/bin/env python
 
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

from ours import *
 
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

		
def exitError(code):
     if code == 0:
         print "OK"
         exit(0)
     print "ERROR:", code
     exit(code)

def initVelma():
     print "Moving to the starting position..."
     velma.moveJoint(ours.q_map_starting, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     error = velma.waitForJoint()
     if error != 0:
         print "The action should have ended without error, but the error code is", error
         exitError(6)
 
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(ours.q_map_starting, js[1], tolerance=0.1):
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
     if not isConfigurationClose(ours.q_map_starting, js[1], tolerance=0.3):
         print "This test requires starting pose:"
         print ours.q_map_starting
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
	 
""" Aktywacja planowania z obiektem w chwytaku """
def virtualObjectRightHand():
     print "Creating a virtual object attached to gripper..."
     # for more details refer to ROS docs for moveit_msgs/AttachedCollisionObject
     object1 = AttachedCollisionObject()
     object1.link_name = "right_HandGripLink"
     object1.object.header.frame_id = "right_HandGripLink"
     object1.object.id = "object1"
     object1_prim = SolidPrimitive()
     object1_prim.type = SolidPrimitive.CYLINDER
     object1_prim.dimensions=[None, None]    # set initial size of the list to 2
     object1_prim.dimensions[SolidPrimitive.CYLINDER_HEIGHT] = 0.23
     object1_prim.dimensions[SolidPrimitive.CYLINDER_RADIUS] = 0.06
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
     return object1

'''def virtualObjectLeftHand():
     print "Creating a virtual object attached to gripper..."
     # for more details refer to ROS docs for moveit_msgs/AttachedCollisionObject
     object1 = AttachedCollisionObject()
     object1.link_name = "left_HandGripLink"
     object1.object.header.frame_id = "left_HandGripLink"
     object1.object.id = "object1"
     object1_prim = SolidPrimitive()
     object1_prim.type = SolidPrimitive.CYLINDER
     object1_prim.dimensions=[None, None]    # set initial size of the list to 2
     object1_prim.dimensions[SolidPrimitive.CYLINDER_HEIGHT] = 0.23
     object1_prim.dimensions[SolidPrimitive.CYLINDER_RADIUS] = 0.06
     object1_pose = pm.toMsg(PyKDL.Frame(PyKDL.Rotation.RotY(math.pi/2)))
     object1.object.primitives.append(object1_prim)
     object1.object.primitive_poses.append(object1_pose)
     object1.object.operation = CollisionObject.ADD
     object1.touch_links = ['left_HandPalmLink',
         'left_HandFingerOneKnuckleOneLink',
         'left_HandFingerOneKnuckleTwoLink',
         'left_HandFingerOneKnuckleThreeLink',
         'left_HandFingerTwoKnuckleOneLink',
         'left_HandFingerTwoKnuckleTwoLink',
         'left_HandFingerTwoKnuckleThreeLink',
         'left_HandFingerThreeKnuckleTwoLink',
         'left_HandFingerThreeKnuckleThreeLink']
     return object1'''

# Wykonanie ruchu z planowaniem
def planAndExecute(q_dest):
 print "Planning motion to the goal position using set of all joints..."
 print "Moving to valid position, using planned trajectory."
 goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
 for i in range(10):
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
	 exitError("THE PLANNING HAS FAILED. PLEASE RESET THE ENVIRONMENT")

#-----------------------------------------------------MAIN---------------------------------------------------------------#
 
if __name__ == "__main__":
     global p
	 global velma

      # mapa stawów do edycji
     q_map_change = {'torso_0_joint':0,
         'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8, 'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85,
         'right_arm_4_joint':0, 'right_arm_5_joint':-0.5, 'right_arm_6_joint':0,
         'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25, 'left_arm_3_joint':-0.85,
         'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
 
 
     #---------------------------------------INITY-----------------------------------------------#
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

# TU BYLO planAndExecute()

     diag = velma.getCoreCsDiag()
     if not diag.motorsReady():
         print "Motors must be homed and ready to use for this test."
         exitError(1)
 
     #---------------------------------------ZABAWA START--------------------------------------------#
 
     toJnp(); #ruch w przestrzeni satwow
 
     #initVelma(); #przejscie do pozycji poczatkowej
 
     #ours.mapBuilding(); #Odkrywanie otoczenia poprzez rozgladanie sie i obracanie


     """Wczytywanie mapy"""
     oml = OctomapListener("/octomap_binary")
     rospy.sleep(1.0)
     octomap = oml.getOctomap(timeout_s=5.0)
     p.processWorld(octomap)
     print "Uwzglednianie mapy"


     """podniesienie reki i obrocenie sie do stolika z piwem"""
     ours.gripper_action("right","grab",0); #chowamy paluszki
     ours.gripper_action("left","grab",0); #chowamy paluszki
     [x_p,y_p,z_p,theta] = findObject("beer") # znajdowanie puszki
     ours.planTorsoAngle(theta); #obliczenie kata
     ours.handsUp(); #edit joints
     planAndExecute(q_map_change); #obracanie torsu do puszki i podnoszenie rak
     ours.gripper_action("right","drop",0); #wystawiamy paluszki
     toCart(); #przejscie do trybu cart_imp
     print "podniesienie reki i obrocenie sie do stolika z piwem"
     

     """ustawienie chwytaka w odleglosci do puszki"""
     rot = PyKDL.Rotation.RPY(0, 0, theta)
     help = math.sqrt((y_p/x_p)*(y_p/x_p)+1)
     if x_p<0:
     	x_new = x_p+ours.D1/help
     else :
     	x_new = x_p-ours.D1/help 
     y_new = (y_p/x_p)*x_new
     B_T = PyKDL.Frame(rot, PyKDL.Vector(x_new,y_new, z_p+0.1)) #tworzenie macierzy jednorodnej do ustawienia chwytaka
     ours.moveCart(B_T); #ruch chwytakiem
     print "ustawienie chwytaka w odleglosci do puszki"


     """ustawienie chwytaka blizej puszki"""
     if x_p<0:
     	x_new = x_p+ours.D2/help
     else :
     	x_new = x_p-ours.D2/help
     y_new = (y_p/x_p)*x_new
     B_T = PyKDL.Frame(rot, PyKDL.Vector(x_new,y_new, z_p+0.1)) #tworzenie macierzy jednorodnej do ustawienia chwytaka
     ours.moveCart(B_T); #ruch chwytakiem
     print "ustawienie chwytaka w blizej puszki"


     """zlapanie piwka i sprawdzenie czy chwycone"""
     toJnp(); #ruch w przestrzeni stawow
     ours.gripper_action("right","grab",1);
     print "zlapanie piwka i sprawdzenie czy chwycone"


     """Podniesienie puszki"""
     toCart(); #przejscie do trybu cart_imp
     B_T = PyKDL.Frame(rot, PyKDL.Vector(x_new,y_new, z_p+0.5)) #tworzenie macierzy jednorodnej do ustawienia chwytaka
     ours.moveCart(B_T); #ruch chwytakiem
     print "Podniesienie puszki"


     """Sprawdzenie czy chwycone"""
     toJnp();
     ours.checkRight([80.0/180.0*math.pi,80.0/180.0*math.pi,80.0/180.0*math.pi,0],1) #zlapanie piwka i sprawdzanie czy chwycone
     actual_joints = velma.getLastJointState() #zapamietanie czasu i aktualnej pozycji stawow
     q_map_change = writeJointStateToQMAP(q_map_change,actual_joints) #wyluskanie pozycji stawow
     print "Sprawdzenie czy chwycone"


     """Tworzenie wirtualnego obiektu i ustawinie katow w stawach"""       
     object1 = virtualObjectRightHand() #tworzenie wirtualnego obiektu
     print "Publishing the attached object marker on topic /attached_objects"
     pub = MarkerPublisherThread(object1)
     pub.start()
     [x_c,y_c,z_c,theta] = ours.findObject("cafe_table") #znalezenie stolika do kawy
     ours.planTorsoAngle(theta); #znajdowanie puszki i obliczenie kata
     print "Tworzenie wirtualnego obiektu i ustawinie katow w stawach"
     

     """Planowanie z piwem w reku"""
     print "Planning motion to the goal position using set of all joints..."
     print "Moving to valid position, using planned trajectory."
     goal_constraint_1 = qMapToConstraints(q_map_change, 0.01, group=velma.getJointGroup("impedance_joints"))
     for i in range(10):
         rospy.sleep(0.5)
         js = velma.getLastJointState()
         print "Planning (try", i, ")..."
         traj = p.plan(js[1], [goal_constraint_1], "impedance_joints", max_velocity_scaling_factor=0.06, planner_id="RRTConnect", attached_collision_objects=[object1])
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
     if not isConfigurationClose(q_map_change, js[1]):
         exitError("THE PLANNING HAS FAILED. PLEASE RESET THE ENVIRONMENT")
     rospy.sleep(1.0)
     print "Planowanie z piwem w reku"


     """Przesuniecie chwytaka z piwem nad drugi stolik"""
     toCart()
     rot = PyKDL.Rotation.RPY(0, 0, theta)
     help = math.sqrt(math.pow(y_c/x_c,2)+1)
     if x_c < 0:
     	x_new = x_c + ours.R/help
     else :
     	x_new = x_c - ours.R/help
     y_new = (y_c/x_c)*x_new
     z_new = H + 0.4 # zmniejszyc??
     B_T = PyKDL.Frame(rot, PyKDL.Vector(x_new, y_new, z_new)) #tworzenie macierzy jednorodnej do ustawienia chwytaka
     ours.moveCart(B_T); #ruch chwytakiem
     print "Przesuniecie chwytaka z piwem nad drugi stolik"


     pub.stop() # Odlaczamy obiekt z chwytaka w przestrzeni planowania


     """Puszczenie piwka"""
     toJnp()
     ours.gripper_action("right","drop",1); #puszczenie piwka
     print "Puszczenie piwka"


     """ Cofniecie reki """
     toCart()
     x_abs =  math.fabs(x_new)
     x_new = (x_abs - 0.12)*(x_new/x_abs)
     y_abs =  math.fabs(y_new)
     y_new = (y_abs - 0.12)*(y_new/y_abs)
     B_T = PyKDL.Frame(rot, PyKDL.Vector(x_new, y_new, z_new)) #tworzenie macierzy jednorodnej do ustawienia chwytaka
     ours.moveCart(B_T); #ruch chwytakiem
     print "Reka cofnieta"


     """Powrot do pozycji poczatkowej"""
	 ours.gripper_action("right","grab",0); #chowamy paluszki
     planAndExecute(ours.q_map_starting)
     print "Powrot do pozycji poczatkowej"

     exitError(0)