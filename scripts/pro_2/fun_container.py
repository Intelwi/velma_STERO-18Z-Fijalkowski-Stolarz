#!/usr/bin/env python

""" ZBIOR FUNKCJI I STALYCH """
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import math
import PyKDL
 
from velma_common import *
from rcprg_ros_utils import exitError

#--------------------------------------------------------------------------------------------#
#-------------------------------------v-"CONSTANTS"-v----------------------------------------#

D1 = 0.5 #odleglosc ustawienia chwytaka od szafki
D2 = 0.07 #odleglosc gripper_jointa od klamki
move_time = 10.0 # stala czasowa (do przemnazania)

# mapa stawow do modyfikacji
q_map_changing = {'torso_0_joint':0,
	'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
	'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
	'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
	'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
	'right_arm_4_joint':0,      'left_arm_4_joint':0,
	'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
	'right_arm_6_joint':0,      'left_arm_6_joint':0 }
	
# mapa stawow w stanie poczatkowym
q_map_starting = {'torso_0_joint':0,
	'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
	'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
	'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
	'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
	'right_arm_4_joint':0,      'left_arm_4_joint':0,
	'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
	'right_arm_6_joint':0,      'left_arm_6_joint':0 }


#--------------------------------------------------------------------------------------------#
#--------------------------------------v-FUNCTIONS-v-----------------------------------------#

def rightHandUp():
	q_map_changing['right_arm_0_joint'] = 1.0
	q_map_changing['right_arm_1_joint'] = -1.1
	q_map_changing['right_arm_3_joint'] = 2
	q_map_changing['right_arm_4_joint'] = 0.2
	
	
def planTorsoAngle(theta):
	if theta>1.55:
		thetaT = 1.55
	elif theta<-1.55:
		thetaT = -1.55
	else:
		thetaT = theta
	q_map_changing['torso_0_joint'] = thetaT
	
	
def findObject(object_id):
	T_B_Jar = velma.getTf("B", object_id) #odebranie pozycji i orientacji obiektu
	rot = T_B_Jar.M #pobranie orientacji szafki
	z = T_B_Jar.p[2]
	y = T_B_Jar.p[1]
	x = T_B_Jar.p[0]

	theta = math.atan2(y,x)
	[R,P,Y]=rot.GetRPY()
	return x,y,z,theta,Y
	
	
def makeWrench(force, torque):
	f = PyKDL.Vector(force[0],force[1],force[2])
	t = PyKDL.Vector(torque[0],torque[1],torque[2])
	return PyKDL.Wrench(f, t)


def countTime(T_B_Trd):
	T_B_Current = velma.getTf("B", "Tr")
	x_diff = T_B_Trd.p[0] - T_B_Current.p[0]
	y_diff = T_B_Trd.p[1] - T_B_Current.p[1]
	z_diff = T_B_Trd.p[2] - T_B_Current.p[2]
	t = math.sqrt(math.pow(x_diff,2)+math.pow(y_diff,2)+math.pow(z_diff,2))
	#print x_diff, y_diff, z_diff
	print "TYLE trza przeleciec: ", t
	return t


def getGripperXYfi():
	T_B_Current = velma.getTf("B", "Gr")
	[e1,e2,fi] = T_B_Current.M.GetRPY()
	return T_B_Current.p[0], T_B_Current.p[1], fi

#--------------------------------------------------------------------------------------------#
#----------------------------------------v-MOVES-v-------------------------------------------#

def moveRightGripper(dest_q): # jnp
	print "fold right gripper's fingers"
	velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
	if velma.waitForHandRight() != 0:
		exitError(10)
	rospy.sleep(0.5)
	if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
		exitError(11)


def moveVelmaJoints(q_map): # jnp
	global move_time
	print "Moving to the position defined in q_map"
	velma.moveJoint(q_map, move_time/1.6, start_time=0.01, position_tol=15.0/180.0*math.pi)
	error = velma.waitForJoint()
	if error != 0:
		print "The action should have ended without error, but the error code is", error
		exitError(6)

	rospy.sleep(0.5)
	js = velma.getLastJointState()
	if not isConfigurationClose(q_map, js[1], tolerance=0.1):
		exitError(10)
		

def moveCart(T_B_Trd): # cart
	global move_time
	t=countTime(T_B_Trd)
	
	print "Moving right wrist to pose defined in world frame..."
	if not velma.moveCartImpRight([T_B_Trd], [move_time*t], None, None, None, None, makeWrench([5,5,5], [5,5,5]), start_time=0.01):
		exitError(13)
	if velma.waitForEffectorRight() != 0:
		exitError(14)
	rospy.sleep(0.5)
	
	print "Calculating difference between desiread and reached pose..."
	T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Tr"), 1.0)
	#print T_B_T_diff
	if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
		exitError(15)
		

def impedStearing(T_B_Trd,imp_list,pt): # cart
	global move_time
	print "Rozpoczecie sterowania impendacyjnego---------------------------"

	# This test/tutorial executes simple impedance commands in Cartesian impedance mode.
	# To see the tool frame add 'tf' in rviz and enable 'right_arm_tool' frame.
	# At every state switch to cart_imp, the tool frames are reset.
	# Also, the tool impedance parameters are reset to 1500N/m in every
	# direction for linear stiffness and to 150Nm/rad in every direction for angular
	# stiffness, i.e. (1500,1500,1500,150,150,150)."

	"""TEST WALNIECIA W SZAFKE"""
	#pt=0.05 # tolerancja velocity (domyslne: none)
	t=countTime(T_B_Trd)
	actual_gripper_position = velma.getTf("B", "Tr") #aktualna pozycja chwytaka
	print "Moving right wrist to pose defined in world frame..."
	if not velma.moveCartImpRight([T_B_Trd], [move_time*t], None, None, imp_list, [0.5], max_wrench=makeWrench([5,5,5], [5,5,5]),
	start_time=0.01, path_tol=PyKDL.Twist(PyKDL.Vector(pt,pt,pt), PyKDL.Vector(pt,pt,pt))):
		exitError(13)
	if velma.waitForEffectorRight() != 0: #zglaszane jak chwytak nie moze osiagnac zadanej pozycji
		#print "Calculating difference between desiread and reached pose..."
		#T_B_T_diff = PyKDL.diff(T_B_Trd, actual_gripper_position, 1.0) #liczenie roznicy w sumie nie potrzebne chyba
		#print T_B_T_diff
		return actual_gripper_position.p[0] , actual_gripper_position.p[1], actual_gripper_position.p[2]
	print "Pose reached, there was no collision"
	return actual_gripper_position.p[0] , actual_gripper_position.p[1], actual_gripper_position.p[2]

	#exitError("Pose reached, there was no collision") #jak sie nie zderzy z niczym to po co ktos ma uzyc impedStearing?
	
#--------------------------------------------------------------------------------------------#
#---------------------------------------v-CONFIGS-v------------------------------------------#

def toCart():
	print "Switch to cart_imp mode (no trajectory)..."
	if not velma.moveCartImpRightCurrentPos(start_time=0.2):
		exitError(10)
	if velma.waitForEffectorRight() != 0:
		exitError(11)
	rospy.sleep(0.5)

	diag = velma.getCoreCsDiag()
	if not diag.inStateCartImp():
		print "The core_cs should be in cart_imp state, but it is not"
		exitError(12)
        
        
def toJnp():
    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(7)
    rospy.sleep(0.5)
    
    diag = velma.getCoreCsDiag()
    if not diag.inStateJntImp():
        print "The core_cs should be in jnt_imp state, but it is not"
        exitError(8)


def initVelma():
	global velma
	rospy.init_node('test_cimp_imp')

	rospy.sleep(0.5)

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
		

def initVelmaPosition():
	global move_time
	print "Moving to the starting position..."
	velma.moveJoint(q_map_starting, move_time, start_time=0.01, position_tol=15.0/180.0*math.pi)
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
	velma.moveHead(q_dest, move_time/2, start_time=0.01)
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

	print "reset left gripper"
	velma.resetHandLeft()
	if velma.waitForHandLeft() != 0:
		exitError(2)
	rospy.sleep(0.5)
	if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), [0,0,0,0]):
		exitError(3)

	print "reset right gripper"
	velma.resetHandRight()
	if velma.waitForHandRight() != 0:
		exitError(4)
	rospy.sleep(0.5)
	if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), [0,0,0,0]):
		exitError(5)

	rospy.sleep(1.0)

