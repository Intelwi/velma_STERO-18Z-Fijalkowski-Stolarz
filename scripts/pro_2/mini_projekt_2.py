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
	#print "Przejscie do pozycji poczatkowej"


	# get initial configuration
	# js_init = velma.getLastJointState()


	"""Znalezienie szafki"""
	[x,y,z,theta,Y] = findObject("cabinet")
	cab_rot = PyKDL.Rotation.RPY(0,0,Y) #obrot szafki
	coords_cabinet = PyKDL.Vector(x,y,z) #wspolrzedne szafki
	print "Znaleziono szafke"


	"""Ustawienie torsu do szafki i uniesienie reki"""
	rightHandUp()
	planTorsoAngle(theta)
	moveVelmaJoints(q_map_changing)
	print "WYKONANO: ustawienie sie do szafki"


	"""Zgiecie palcow prawego chytaka"""
	dest_q = [90.0/180.0*math.pi, 90.0/180.0*math.pi, 90.0/180.0*math.pi, math.pi]
	moveRightGripper(dest_q)
	print "WYKONANO: zgiecie palcow prawego chytaka"


	"""Przejscie do trybu cart_imp"""
	toCart()
	print "Przejscie do trybu cart_imp"

	
	"""Utworzenie macierzy jednorodnej dla chwytaka - pozycja poczatkowa"""
	x_relative = 0.5 
	y_relative = 0.12
	z_relative = 0.13
	init_vector = PyKDL.Vector(x_relative, y_relative, z_relative) #wektor poczatkowy
	final_vector = cab_rot*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	gripper_rot = PyKDL.Rotation.RPY(0,0,Y-math.pi)	#obrot chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	print "UTWORZONO: macierz jednorodna dla chwytaka - pozycja poczatkowa"

	"""Ustawienie chwytaka blisko szafki"""
	moveCart(T_B_Trd)
	print "WYKONANO: ustawienie chwytaka blisko szafki"
	rospy.sleep(0.5)


	"""Utworzenie macierzy jednorodnej dla chwytaka by uderzyl w szafke"""
	x_relative = 0.3
	init_vector = PyKDL.Vector(x_relative, y_relative, z_relative) #wektor poczatkowy
	final_vector = cab_rot*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	print "UTWORZONO: macierz jednorodna dla chwytaka by uderzyl w szafke"

	"""Walniecie w szafke"""
	imped = makeWrench([350,350,350],[50,50,50]),
	[x_g,y_g,z_g]=impedStearing(T_B_Trd,imped) #zwraca aktualne polozenie chwytaka
	print "WYKONANO: test walniecia w szafke"
	rospy.sleep(0.5)


	"""Utworzenie macierzy jednorodnej dla chwytaka by lekko cofnal reke"""
	x_relative = 0.43
	init_vector = PyKDL.Vector(x_relative, y_relative, z_relative) #wektor poczatkowy
	final_vector = cab_rot*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	print "UTWORZONO: macierz jednorodna dla chwytaka by lekko cofnal reke"

	"""Lekkie cofnancie reki"""
	moveCart(T_B_Trd)
	print "WYKONANO: lekkie cofnancie reki"
	rospy.sleep(0.5)

	#TO JUZ w miare DZIALA -------------WIEM CZEMU----------------------------------------------------------------------------------------------

	"""Wyliczenie punktu do ktorego ma poruszyc reka zeby trafic w uchwyt (doswiadczalne y, wyliczamy x), mamy kat orientacji szafki, wiec mozna sobie wyliczyc prosta laczaca obecne polozenie chwytaka i to ktore bedzie przy uchwycie"""
	y_relative=-0.03 #doswiadczalne y tak by chwytak uderzyl w uchwyt
	#x_relative = (y_new - y_g + x_g*math.tan(Y))/math.tan(Y) #wyliczenie x_new dla y_new tak by chwytak poruszal sie wzdluz drzwiczek szafki
	init_vector = PyKDL.Vector(x_relative, y_relative, z_relative)
	final_vector = cab_rot*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka do uderzenia w uchwyt
	print "UTWORZONO: macierz jednorodna dla chwytaka by uderzyl w uchwyt szafki"

	"""Dojechanie do uchwytu"""
	[x_g,y_g,z_g]=impedStearing(T_B_Trd,imped) #zwraca aktualne polozenie chwytaka
	print "WYKONANO: dojechanie do uchwytu"
	rospy.sleep(0.5)


	"""Utworzenie macierzy jednorodnej dla chwytaka by pociagnal drzwi szafki do siebie"""
	x_relative = 0.7
	init_vector = PyKDL.Vector(x_relative, y_relative, z_relative) #wektor poczatkowy
	final_vector = cab_rot*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	gripper_rot = PyKDL.Rotation.RPY(0,0,Y-math.pi-0.2)	#obrot chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	print "UTWORZONO: macierz jednorodna dla chwytaka by pociagnal drzwi szafki"

	"""Pociagniecie drzwiczek"""
	[x_g,y_g,z_g]=impedStearing(T_B_Trd,imped) #zwraca aktualne polozenie chwytaka
	print "WYKONANO: pociagniecie drzwiczek"
	rospy.sleep(0.5)
	
	
	"""Utworzenie macierzy jednorodnej dla chwytaka by wyswobodzil lape"""
	x_relative = 0.45
	y_relative = 0.4
	init_vector = PyKDL.Vector(x_relative, y_relative, z_relative) #wektor poczatkowy
	final_vector = cab_rot*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	#gripper_rot = PyKDL.Rotation.RPY(0,0,Y-math.pi+1.0)	#obrot chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	print "UTWORZONO: macierz jednorodna dla chwytaka by wyswobodzil lape"

	"""Wyswobodzenie lapy"""
	[x_g,y_g,z_g]=impedStearing(T_B_Trd,imped) #zwraca aktualne polozenie chwytaka
	print "WYKONANO: wyswobodzenie lapy"
	rospy.sleep(0.5)
	

	"""Przejscie do trybu jnp_imp"""
	toJnp()
	print "Przejscie do trybu jnp_imp"


	"""Powrot do pozycji poczatkowej"""
	moveVelmaJoints(q_map_starting)
	print "WYKONANO: powrot do pozycji poczatkowej"


	exitError(0)
 
