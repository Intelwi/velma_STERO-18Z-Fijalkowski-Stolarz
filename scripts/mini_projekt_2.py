#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
 
from velma_common import *
from rcprg_ros_utils import exitError
 

def toCart():
     print "Switch to cart_imp mode (no trajectory)..."
     if not velma.moveCartImpRightCurrentPos(start_time=0.2):
         exitError(10)
     if velma.waitForEffectorRight() != 0:
         exitError(11)

def toJnp():
     print "Switch to jnt_imp mode (no trajectory)..."
     velma.moveJointImpToCurrentPos(start_time=0.2)
     error = velma.waitForJoint()
     if error != 0:
         print "The action should have ended without error, but the error code is", error
         exitError(7)

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
     if not isConfigurationClose(q_map_starting, js[1], tolerance=0.2):
         print "This test requires starting pose:"
         print q_map_starting
         exitError(9)

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

def moveVelmaJoints(q_map):
     print "Moving to the next position..."
     velma.moveJoint(q_map, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     error = velma.waitForJoint()
     if error != 0:
         print "The action should have ended without error, but the error code is", error
         exitError(6)
 
     rospy.sleep(0.5)
     js = velma.getLastJointState()
     if not isConfigurationClose(q_map, js[1], tolerance=0.1):
    	 exitError(10)

if __name__ == "__main__":
     # define some configurations
     q_map_starting = {'torso_0_joint':0,
         'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
         'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
         'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
         'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
         'right_arm_4_joint':0,      'left_arm_4_joint':0,
         'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
         'right_arm_6_joint':0,      'left_arm_6_joint':0 }
 
     q_map_1 = {'torso_0_joint':0.0,
         'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
         'right_arm_1_joint':-1.57,  'left_arm_1_joint':1.57,
         'right_arm_2_joint':1.57,   'left_arm_2_joint':-1.57,
         'right_arm_3_joint':1.57,   'left_arm_3_joint':-1.7,
         'right_arm_4_joint':0.0,    'left_arm_4_joint':0.0,
         'right_arm_5_joint':-1.57,  'left_arm_5_joint':1.57,
         'right_arm_6_joint':0.0,    'left_arm_6_joint':0.0 }
 
     rospy.init_node('test_cimp_imp')
 
     rospy.sleep(0.5)
 
     print "This test/tutorial executes simple impedance commands"\
         " in Cartesian impedance mode.\n"
 
     print "Running python interface for Velma..."
     velma = VelmaInterface()
     print "Waiting for VelmaInterface initialization..."
     if not velma.waitForInit(timeout_s=10.0):
         print "Could not initialize VelmaInterface\n"
         exitError(1)
     print "Initialization ok!\n"
 
     if velma.enableMotors() != 0:
         exitError(2)
 
     diag = velma.getCoreCsDiag()
     if not diag.motorsReady():
         print "Motors must be homed and ready to use for this test."
         exitError(3)
 
     def makeWrench(lx,ly,lz,rx,ry,rz):
         return PyKDL.Wrench(PyKDL.Vector(lx,ly,lz), PyKDL.Vector(rx,ry,rz))
     

     """Przejscie do trybu jnp_imp"""
     toJnp()
     print "Przejscie do trybu jnp_imp"

 
     rospy.sleep(0.5)
     diag = velma.getCoreCsDiag()
     if not diag.inStateJntImp():
         print "The core_cs should be in jnt_imp state, but it is not"
         exitError(8)
 

     """Przejscie do pozycji poczatkowej"""
     initVelma()
     print "Przejscie do pozycji poczatkowej"
 

     # get initial configuration
     js_init = velma.getLastJointState()


     """Przejscie do trybu cart_imp"""
     toCart()
     print "Przejscie do trybu cart_imp"
 

     rospy.sleep(0.5)

 
     diag = velma.getCoreCsDiag()
     if not diag.inStateCartImp():
         print "The core_cs should be in cart_imp state, but it is not"
         exitError(12)



     """Wykonanie testu sterowania impedancyjnego""" 

     print "Rozpoczecie testu sterowania impendacyjnego"
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
         exitError(15)
 
     print "Set impedance to (1000,1000,125,150,150,150) in tool frame."
     imp_list = [makeWrench(1000,1000,1000,150,150,150),
                 makeWrench(1000,1000,500,150,150,150),
                 makeWrench(1000,1000,250,150,150,150),
                 makeWrench(1000,1000,125,150,150,150)]
     if not velma.moveCartImpRight(None, None, None, None, imp_list, [0.5,1.0,1.5,2.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
         exitError(16)
     if velma.waitForEffectorRight() != 0:
         exitError(17)
 
     rospy.sleep(1.0)
 
     print "Set impedance to (1000,1000,1000,150,150,150) in tool frame."
     imp_list = [makeWrench(1000,1000,250,150,150,150),
                 makeWrench(1000,1000,500,150,150,150),
                 makeWrench(1000,1000,1000,150,150,150)]
     if not velma.moveCartImpRight(None, None, None, None, imp_list, [0.5,1.0,1.5], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
         exitError(16)
     if velma.waitForEffectorRight() != 0:
         exitError(17)
 

     print "Zakonczenie testu sterowania impedancyjnego"


     """Powrot do pozycji poczatkowej"""
     moveVelmaJoints(q_map_starting)
     print "Powrot do pozycji poczatkowej"
 
     exitError(0)
 

