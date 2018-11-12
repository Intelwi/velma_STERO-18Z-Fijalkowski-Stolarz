#!/usr/bin/env python

""" MINI PROJECT 2 """
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import math
import PyKDL
 
from velma_common import *
from rcprg_ros_utils import exitError
from fun_container import *

def rightHandUp():
	q_map_change['right_arm_0_joint'] = 1.1
	q_map_change['right_arm_1_joint'] = -1.3
	q_map_change['right_arm_3_joint'] = 2
	q_map_change['right_arm_4_joint'] = 0.2

def planTorsoAngle(theta):
	if theta>1.55:
		thetaT = 1.55
	elif theta<-1.55:
		thetaT = -1.55
	else:
		thetaT = theta
	q_map_change['torso_0_joint'] = thetaT

if __name__ == "__main__":
	
     # mapa stawow do modyfikacji
     q_map_change = {'torso_0_joint':0,
         'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
         'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
         'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
         'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
         'right_arm_4_joint':0,      'left_arm_4_joint':0,
         'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
         'right_arm_6_joint':0,      'left_arm_6_joint':0 }

     """Uruchomienie interfejsu Velmy i sprawdzenie silnikow"""
     initVelma()
     

     """Przejscie do trybu jnp_imp"""
     toJnp()
     print ("Przejscie do trybu jnp_imp")
 
 
     """Przejscie do pozycji poczatkowej"""
     initVelmaPosition()
     print "Przejscie do pozycji poczatkowej"
 

     # get initial configuration
     # js_init = velma.getLastJointState()

     """Znalezienie szafki"""
     [x,y,z,theta]=findObject("cabinet")
     print ("Znalezienie szafki")


     """Ustawienie sie do szafki"""
     rightHandUp()
     planTorsoAngle(theta)
     moveVelmaJoints(q_map_change)
     print ("Ustawienie sie do szafki")

    
     """Zgiecie palcow prawego chytaka"""
     dest_q = [90.0/180.0*math.pi,90.0/180.0*math.pi,90.0/180.0*math.pi,180.0/180.0*math.pi]
     moveRightGripper(dest_q)
     print ("Zgiecie palcow prawego chytaka")


     """Przejscie do trybu cart_imp"""
     toCart()
     print ("Przejscie do trybu cart_imp")


     """Ustawienie frame by ustawic chwytak w odleglosci do szafki"""
     rot = PyKDL.Rotation.RPY(0, 0, theta)
     mian = math.sqrt((y/x)*(y/x)+1)
     if x<0:
     	x_new = x+D1/mian #ustawienie chwytaka w odstepie od szafki
     else :
     	x_new = x-D1/mian #ustawienie chwytaka w odstepie od szafki  
     y_new = (y/x)*x_new #rownanie prostej
     T_B_Trd = PyKDL.Frame(rot, PyKDL.Vector(x_new,y_new, z)) #tworzenie macierzy jednorodnej do ustawienia chwytaka
     print "Ustawienie frame by ustawic chwytak w odleglosci do szafki"


     """Wykonanie testu sterowania impedancyjnego"""
     impedStearing(T_B_Trd)


     """Przejscie do trybu jnp_imp"""
     toJnp()
     print "Przejscie do trybu jnp_imp"


     """Powrot do pozycji poczatkowej"""
     moveVelmaJoints(q_map_starting)
     print "Powrot do pozycji poczatkowej"

     exitError(0)
 

