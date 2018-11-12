#!/usr/bin/env python

# polecam polecenie: sudo apt-get install geany
""" MINI PROJECT 2 """
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import math
import PyKDL
 
from velma_common import *
from rcprg_ros_utils import exitError
from fun_container import *

if __name__ == "__main__":
	
	"""Uruchomienie interfejsu Velmy i sprawdzenie silnikow"""
	initVelma()


	"""Przejscie do trybu jnp_imp"""
	toJnp()
	print "Przejscie do trybu jnp_imp"


	"""Przejscie do pozycji poczatkowej"""
	initVelmaPosition()
	print "Przejscie do pozycji poczatkowej"


	# get initial configuration
	# js_init = velma.getLastJointState()


	"""Znalezienie szafki"""
	[x,y,z,theta] = findObject("cabinet")
	print "Znalezienie szafki"


	"""Ustawienie torsu do szafki i uniesienie reki (po co???)"""
	rightHandUp()
	planTorsoAngle(theta)
	moveVelmaJoints(q_map_changing)
	print "Ustawienie sie do szafki"


	"""Zgiecie palcow prawego chytaka"""
	dest_q = [90.0/180.0*math.pi, 90.0/180.0*math.pi, 90.0/180.0*math.pi, math.pi]
	moveRightGripper(dest_q)
	print "Zgiecie palcow prawego chytaka"


	"""Przejscie do trybu cart_imp"""
	toCart()
	print "Przejscie do trybu cart_imp"


	"""Ustawienie frame by ustawic chwytak w odleglosci D1 do szafki"""
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
 
