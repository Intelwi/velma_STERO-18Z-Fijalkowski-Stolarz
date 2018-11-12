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
	q_map_changing['right_arm_0_joint'] = 1.1
	q_map_changing['right_arm_1_joint'] = -1.3
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

	z = T_B_Jar.p[2]
	y = T_B_Jar.p[1]
	x = T_B_Jar.p[0]

	theta = math.atan2(y,x)

	return x,y,z,theta
	
	
def makeWrench(force, torque):
	f = PyKDL.Vector(force[0],force[1],force[2])
	t = PyKDL.Vector(torque[0],torque[1],torque[2])
	return PyKDL.Wrench(f, t)


#--------------------------------------------------------------------------------------------#
#----------------------------------------v-MOVES-v-------------------------------------------#

def moveRightGripper(dest_q): # cart
	print "move right gripper to:", dest_q
	velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
	if velma.waitForHandRight() != 0:
		exitError(10)
	rospy.sleep(0.5)
	if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
		exitError(11)


def moveVelmaJoints(q_map): # inp
	print "Moving to the position defined in q_map"
	velma.moveJoint(q_map, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
	error = velma.waitForJoint()
	if error != 0:
		print "The action should have ended without error, but the error code is", error
		exitError(6)

	rospy.sleep(0.5)
	js = velma.getLastJointState()
	if not isConfigurationClose(q_map, js[1], tolerance=0.1):
		exitError(10)
		

def impedStearing(T_B_Trd): # cart?
	print "Rozpoczecie testu sterowania impendacyjnego------------------"

	# This test/tutorial executes simple impedance commands in Cartesian impedance mode.
	# To see the tool frame add 'tf' in rviz and enable 'right_arm_tool' frame.
	# At every state switch to cart_imp, the tool frames are reset.
	# Also, the tool impedance parameters are reset to 1500N/m in every
	# direction for linear stiffness and to 150Nm/rad in every direction for angular
	# stiffness, i.e. (1500,1500,1500,150,150,150)."
		
	print "Moving right wrist to pose defined in world frame..."
	#T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.7 , -0.3 , 1.3 ))
	if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, makeWrench([5,5,5], [5,5,5]), start_time=0.5):
		exitError(13)
	if velma.waitForEffectorRight() != 0:
		exitError(14)
	rospy.sleep(0.5)
	
	print "Calculating difference between desiread and reached pose..."
	T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Tr"), 1.0)
	print T_B_T_diff
	if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
		exitError(15)

	#TEST DODANY ------- Przesun reke w bok
	print "przesuniecie reki"
	[x,y,z,theta] = findObject("cabinet")
	rot = PyKDL.Rotation.RPY(0, 0, theta)
	T_B_Trd = PyKDL.Frame(rot, PyKDL.Vector(x-0.5,y-0.07,z+0.16)) #tworzenie macierzy jednorodnej do ustawienia chwytaka bylo
	if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, makeWrench([5,5,5], [5,5,5]), start_time=0.5):
		exitError(13)
	if velma.waitForEffectorRight() != 0:
		exitError(14)
	rospy.sleep(0.5)
	
	print "Calculating difference between desiread and reached pose..."
	T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Tr"), 1.0)
	print T_B_T_diff
	if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
		exitError(15)
	#TEST DODANY ------ KONIEC

	
	print "Set impedance to (1000,1000,125,150,150,150) in tool frame."
	imp_list = [makeWrench([1000,1000,1000],[150,150,150]),
		makeWrench([1000,1000,500],[150,150,150]),
		makeWrench([1000,1000,250],[150,150,150]),
		makeWrench([1000,1000,125],[150,150,150])]
	if not velma.moveCartImpRight(None, None, None, None, imp_list, [0.5,1.0,1.5,2.0], makeWrench([5,5,5], [5,5,5]), start_time=0.5):
		exitError(16)
	if velma.waitForEffectorRight() != 0:
		exitError(17) 
	rospy.sleep(1.0)
	

	#----------------walnij w szafe
	"""
	[x,y,z,theta] = findObject("cabinet")
	rot = PyKDL.Rotation.RPY(0, 0, theta)
	T_B_Trd = PyKDL.Frame(rot, PyKDL.Vector(x-0.3,y,z)) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	print "Moving right wrist to pose defined in world frame..."
	#T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.7 , -0.3 , 1.3 ))
	a=5
	b=0.5
	if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, makeWrench([a,a,a], [a,a,a]), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(b,b,b), PyKDL.Vector(b,b,b))):
		exitError(13)
	if velma.waitForEffectorRight(timeout_s = 1) != 0:  # ODE Message 3: LCP internal error, s <= 0 (s=-0.0000e+00)
		exitError(14)
	rospy.sleep(0.5)
	
	print "Calculating difference between desiread and reached pose..."
	T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Tr"), 1.0)
	print T_B_T_diff"""
	#--------------------------------








	""" W tescie dodanym zostalo uzyte velma.waitForEffectorRight(timeout_s = x) gdzie velma przestaje napierac na szafke po czasie x. To nie dziala bo trzeba dobierac czas x. Ciekawe bo argument timeout_s wcalenie nie jest wymagany. Trzeba jakos uzyc T_B_T_diff = PyKDL.diff(T_B_first, velma.getTf("B", "Tr"), 1.0)."""
	

	#TEST DODANY -------OTWIERANIE SZAFKI ZLA METODA--------------------------------------------------------------------------------
	#----------------walnij w szafe
	T_B_Trd = PyKDL.Frame(rot, PyKDL.Vector(x-0.4,y-0.07,z+0.16)) #tworzenie macierzy jednorodnej do ustawienia chwytaka bylo
	print "Moving right wrist to pose defined in world frame..."
	#T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.7 , -0.3 , 1.3 ))
	a=5
	b=0.5
	if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, makeWrench([a,a,a], [a,a,a]), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(b,b,b), PyKDL.Vector(b,b,b))):
		exitError(13)
	if velma.waitForEffectorRight(timeout_s = 3) != 0:  # ODE Message 3: LCP internal error, s <= 0 (s=-0.0000e+00)
		print "Walnelam w szafke!"
	rospy.sleep(0.5)

	#Znalezienie klampki
	T_B_Trd = PyKDL.Frame(rot, PyKDL.Vector(x-0.4,y,z+0.16)) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	print "Moving right wrist to pose defined in world frame..."
	if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, makeWrench([a,a,a], [a,a,a]), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(b,b,b), PyKDL.Vector(b,b,b))):
		exitError(13)
	if velma.waitForEffectorRight(timeout_s = 3) != 0:  # ODE Message 3: LCP internal error, s <= 0 (s=-0.0000e+00)
		print "Znalazlam klamke!"
	rospy.sleep(0.5)

	#Pociagniecie
	T_B_Trd = PyKDL.Frame(rot, PyKDL.Vector(x-0.6,y,z+0.16)) #tworzenie macierzy jednorodnej do ustawienia chwytaka bylo
	print "Moving right wrist to pose defined in world frame..."
	if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, makeWrench([a,a,a], [a,a,a]), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(b,b,b), PyKDL.Vector(b,b,b))):
		exitError(13)
	if velma.waitForEffectorRight(timeout_s = 3) != 0:  # ODE Message 3: LCP internal error, s <= 0 (s=-0.0000e+00)
		print "Znalazlam drugi punkt okregu!"
	rospy.sleep(0.5)
	
	print "Calculating difference between desiread and reached pose..."
	T_B_T_diff = PyKDL.diff(T_B_first, velma.getTf("B", "Tr"), 1.0)
	print T_B_T_diff
	#--------------------------------
	#TEST DODANY -----KONIEC---------------------------------------------------------------------------------------------
	

	

	# ~ print "Set impedance to (1000,1000,1000,150,150,150) in tool frame."
	# ~ imp_list = [makeWrench([1000,1000,250],[150,150,150]),
		# ~ makeWrench([1000,1000,500],[150,150,150]),
		# ~ makeWrench([1000,1000,1000],[150,150,150])]
	# ~ if not velma.moveCartImpRight(None, None, None, None, imp_list, [0.5,1.0,1.5], makeWrench([5,5,5], [5,5,5]), start_time=0.5):
		# ~ exitError(16)
	# ~ if velma.waitForEffectorRight() != 0:
		# ~ exitError(17)
	# ~ rospy.sleep(1.0) #?
	  
	print "Zakonczenie testu sterowania impedancyjnego"
	
	
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
     
