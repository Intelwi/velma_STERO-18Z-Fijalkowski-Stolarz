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

#polozenie i orientacja szafki --- zmienne
#x=y=z=theta=0

#rot = PyKDL.Rotation.RPY(0,0,0)

if __name__ == "__main__":
	global x,y,z,theta,Y
	"""Uruchomienie interfejsu Velmy i sprawdzenie silnikow"""
	initVelma()


	"""Przejscie do trybu jnp_imp"""
	toJnp()
	print "Przejscie do trybu jnp_imp"


	"""Przejscie do pozycji poczatkowej"""
	#initVelmaPosition()
	print "Przejscie do pozycji poczatkowej"


	# get initial configuration
	# js_init = velma.getLastJointState()


	"""Znalezienie szafki"""
	[x_,y_,z_,theta_,Y_] = findObject("cabinet")
	x = x_
	y = y_
	z = z_
	theta = theta_
	Y = Y_
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

	
	"""Utworzenie macierzy jednorodnej dla chwytaka"""
	x_new = 0.5 
	y_new = 0.12
	z_new = 0.13
	init_vector = PyKDL.Vector(x_new,y_new,z_new) #wektor poczatkowy
	coords_cabinet = PyKDL.Vector(x,y,z) #wspolrzedne szafki
	final_vector = PyKDL.Rotation.RPY(0,0,Y)*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	gripper_rot = PyKDL.Rotation.RPY(0,0,Y-math.pi)	#obrot chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	print "Utworzenie macierzy jednorodnej dla chwytaka"


	"""Ustawienie chwytaka blisko szafki"""
	moveCart(T_B_Trd)
	print "Ustawienie chwytaka blisko szafki"


	"""Utworzenie macierzy jednorodnej dla chwytaka by uderzyl w szafke"""
	x_new = 0.4
	init_vector = PyKDL.Vector(x_new,y_new,z_new) #wektor poczatkowy
	final_vector = PyKDL.Rotation.RPY(0,0,Y)*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	print "Utworzenie macierzy jednorodnej dla chwytaka by uderzyl w szafke"


	"""Wykonanie testu sterowania impedancyjnego"""
	[x_g,y_g,z_g]=impedStearing(T_B_Trd) #zwraca aktualne polozenie chwytaka
	print "Wykonanie testu sterowania impedancyjnego"

	"""Utworzenie macierzy jednorodnej dla chwytaka by lekko cofnal reke"""
	x_new = 0.43
	init_vector = PyKDL.Vector(x_new,y_new,z_new) #wektor poczatkowy
	final_vector = PyKDL.Rotation.RPY(0,0,Y)*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	print "Utworzenie macierzy jednorodnej dla chwytaka by lekko cofnal reke"

	
	"""Lekkie cofnancie reki"""
	moveCart(T_B_Trd)
	print "Lekkie cofnancie reki"

	#TO JUZ NIE DZIALA -------------NIE WIEM CZEMU----------------------------------------------------------------------------------------------

	"""Wyliczenie punktu do ktorego ma poruszyc reka zeby trafic w uchwyt (doswiadczalne y, wyliczamy x), mamy kat orientacji szafki, wiec mozna sobie wyliczyc prosta laczaca obecne polozenie chwytaka i to ktore bedzie przy uchwycie"""
	y_new=0.07 #doswiadczalne y tak by chwytak uderzyl w uchwyt
	x_new = (y_new - y_g + x_g*math.tan(Y))/math.tan(Y) #wyliczenie x_new dla y_new tak by chwytak poruszal sie wzdluz drzwiczek szafki
	final_vector = PyKDL.Vector(x_new,y_new,z_new)
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka do uderzenia w uchwyt
	print "Utworzenie macierzy jednorodnej dla chwytaka by uderzyl w uchwyt szafki poruszajac chwytakiem wzdluz drzwiczek"

	"""Wykonanie testu sterowania impedancyjnego"""
	[x_g,y_g,z_g]=impedStearing(T_B_Trd) #zwraca aktualne polozenie chwytaka
	print "Wykonanie testu sterowania impedancyjnego"


	"""Przejscie do trybu jnp_imp"""
	toJnp()
	print "Przejscie do trybu jnp_imp"


	"""Powrot do pozycji poczatkowej"""
	moveVelmaJoints(q_map_starting)
	print "Powrot do pozycji poczatkowej"


	exitError(0)
 
