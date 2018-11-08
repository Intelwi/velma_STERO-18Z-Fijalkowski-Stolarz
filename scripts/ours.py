#!/usr/bin/env python
from main import *

D1 = 0.35
D2 = 0.25
R = 0.913/2 
H = 0.82

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

""" Sprawdzanie czy w chwytaku jest obiekt """
def checkRight(dest_q,isBeer):
    	if not isHandConfigurationClose(main.velma.getHandRightCurrentConfiguration(), dest_q):
        	if dest_q == [80.0/180.0*math.pi,80.0/180.0*math.pi,80.0/180.0*math.pi,0] and isBeer==1:	
        		print "Beer is grabbed!"
        	else:	
        		print main.velma.getHandLeftCurrentConfiguration(), dest_q
  			exitError(9)
        else:
        	if dest_q == [80.0/180.0*math.pi,80.0/180.0*math.pi,80.0/180.0*math.pi,0] and isBeer==1:
       			print "Beer is not grabbed! Going back to starting position."
       			main.planAndExecute(q_map_starting)	
        		exitError("Task execution failed. Came back to starting position correctly.")
				
'''def checkLeft(dest_q,isBeer):
    	if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), dest_q):
        	if dest_q == [80.0/180.0*math.pi,80.0/180.0*math.pi,80.0/180.0*math.pi,0] and isBeer==1:	
        		print "Beer is grabbed!"
        	else:	
        		print velma.getHandLeftCurrentConfiguration(), dest_q
  			exitError(7)
        else:
        	if dest_q == [80.0/180.0*math.pi,80.0/180.0*math.pi,80.0/180.0*math.pi,0] and isBeer==1:
       			print "Beer is not grabbed! Going back to starting position."
       			planAndExecute(q_map_starting)	
        		exitError("Task execution failed. Came back to starting position correctly.")'''			

		
""" Kontrola akcji chwytaka: chwyt / puszczenie """
def gripper_action(gripper,action,isBeer):
     if gripper == "left":	
     	if action == "grab":
     		dest_q = [80.0/180.0*math.pi,80.0/180.0*math.pi,80.0/180.0*math.pi,0]
     		left_gripper_action(dest_q,isBeer)
     	elif action == "drop":
     		dest_q = [0,0,0,0]
     		left_gripper_action(dest_q,isBeer)
	else:
		return
     	rospy.sleep(1)
     
     elif gripper == "right":	
     	if action == "grab":
     		dest_q = [80.0/180.0*math.pi,80.0/180.0*math.pi,80.0/180.0*math.pi,0]
     		right_gripper_action(dest_q,isBeer)
     		
     	elif action == "drop":
     		dest_q = [0,0,0,0]
     		right_gripper_action(dest_q,isBeer)
	else:
		return
     	rospy.sleep(1)
     else:
     	return

def findObject(object): #trzeba poprawic by zawsze sie obracal
     #main.velma.waitForInit()
     T_B_Jar = main.velma.getTf("B", object) #odebranie pozycji i orientacji obiektu
   
     z = T_B_Jar.p[2]#z
     y = T_B_Jar.p[1]#y
     x = T_B_Jar.p[0]#x

     theta = math.atan2(y,x)

     return x,y,z,theta


def planTorsoAngle(theta):
     if theta>1.55:
	thetaT = 1.55
     elif theta<-1.55:
	thetaT = -1.55
     else:
        thetaT = theta
     q_map_change['torso_0_joint'] = thetaT
	 
#--------------------------------------------------MOVEMENTS--------------------------------------------------#

""" Wykonanie ruchu chwytaka """			
def right_gripper_action(dest_q,isBeer):
     	print "move right:", dest_q
     	main.velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
     	if main.velma.waitForHandRight() != 0:
        	exitError(6)
      	rospy.sleep(0.5)
     	checkRight(dest_q,isBeer)
		
'''def left_gripper_action(dest_q,isBeer):
     	print "move left:", dest_q
     	velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
     	if velma.waitForHandLeft() != 0:
        	exitError(6)
      	rospy.sleep(0.5)
    	checkLeft(dest_q,isBeer)'''

def moveCart(B_T):
     print "Zaczynam ruch nadgarstka"
     if not main.velma.moveCartImpRight([B_T], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
         exitError(16)
     if main.velma.waitForEffectorRight() != 0:
         exitError(17)
     rospy.sleep(0.5)
     print "calculating difference between desiread and reached pose..."
     T_B_T_diff = PyKDL.diff(B_T, main.velma.getTf("B", "Tr"), 1.0)
     print T_B_T_diff
     if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
         exitError(10)
		 
def handsUp():
     main.q_map_change['right_arm_0_joint'] = 1
     main.q_map_change['right_arm_1_joint'] = -1.2
     main.q_map_change['right_arm_3_joint'] = 2
	 

def moveBody(q_map):
     main.velma.moveJoint(q_map, 8.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     error = main.velma.waitForJoint()
     if error != 0:
         print "The action should have ended without error, but the error code is", error
         exitError(10)
 
     rospy.sleep(0.5)
     js = main.velma.getLastJointState()
     if not isConfigurationClose(q_map, js[1], tolerance=0.1):
         exitError(10)

def moveHead(q_dest):
     main.velma.moveHead(q_dest, 3.0, start_time=0.5)
     if main.velma.waitForHead() != 0:
         exitError(4)
     rospy.sleep(0.5)
     if not isHeadConfigurationClose(main.velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
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