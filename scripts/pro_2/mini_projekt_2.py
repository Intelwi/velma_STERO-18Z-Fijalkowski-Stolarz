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
	dest_q = [77.0/180.0*math.pi, 77.0/180.0*math.pi, 77.0/180.0*math.pi, math.pi]
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
	lk=500
	ak=70
	imped = makeWrench([lk,lk,lk],[ak,ak,ak]),
	[x_g,y_g,z_g]=impedStearing(T_B_Trd,imped,0.05) #zwraca aktualne polozenie chwytaka
	x_g_relative = (PyKDL.Rotation.RPY(0,0,-Y)*(PyKDL.Vector(x_g,y_g,z_g)-coords_cabinet)).x() # wzgledny wektor polozenia przy uderzeniu
	#print "x_g_relative:", x_g_relative
	print "WYKONANO: test walniecia w szafke"
	rospy.sleep(0.5)


	"""Utworzenie macierzy jednorodnej dla chwytaka by lekko cofnal reke"""
	x_relative = x_g_relative + 0.02
	init_vector = PyKDL.Vector(x_relative, y_relative, z_relative) #wektor poczatkowy
	final_vector = cab_rot*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	print "UTWORZONO: macierz jednorodna dla chwytaka by lekko cofnal reke"

	"""Lekkie cofnancie reki"""
	moveCart(T_B_Trd)
	print "WYKONANO: lekkie cofnancie reki"
	rospy.sleep(0.5)
	

	"""Wyliczenie punktu do ktorego ma poruszyc reka zeby trafic w uchwyt (doswiadczalne y)"""
	y_relative=-0.05 #doswiadczalne y tak by chwytak uderzyl w uchwyt
	init_vector = PyKDL.Vector(x_relative, y_relative, z_relative)
	final_vector = cab_rot*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka do uderzenia w uchwyt
	print "UTWORZONO: macierz jednorodna dla chwytaka by uderzyl w uchwyt szafki"

	"""Dojechanie do uchwytu"""
	lk=800
	ak=50
	imped = makeWrench([lk,lk,lk],[ak,ak,ak]),
	[x_g,y_g,z_g]=impedStearing(T_B_Trd,imped,0.028) #zwraca aktualne polozenie chwytaka
	print "WYKONANO: dojechanie do uchwytu"
	rospy.sleep(0.5)

	"""Wyliczenie sily zeby cofnal reke"""
	lk=100
	ak=50
	imped = makeWrench([lk,lk,lk],[ak,ak,ak]),

	y_relative=y_relative+0.02 #doswiadczalne y tak by chwytak sie cofnal
	init_vector = PyKDL.Vector(x_relative, y_relative, z_relative)
	final_vector = cab_rot*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka do odsuniecia
	[x_g1,y_g1,z_g]=impedStearing(T_B_Trd,imped,0.028) #zwraca aktualne polozenie chwytaka
	print "WYKONANO: cofniecie punk 1: ", x_g1, y_g1
	print "Zmiejszono sile"




	#POPRZEDNIE LICZENIE---------------------------------------------
	"""pobranie punktu nr 1
	[x_p1, y_p1, fi] = getGripperXYfi()
	tnafi = math.tan(abs(fi))
	temp = D2/math.sqrt(1+tnafi*tnafi)
	x_p1 = (abs(x_p1)+temp)*(abs(x_p1)/x_p1)
	y_p1 = (abs(y_p1)+temp*tnafi)*(abs(y_p1)/y_p1)
"""
	#POPRZEDNIE LICZENIE KONIEC---------------------------------------------



	"""Utworzenie macierzy jednorodnej dla chwytaka by pociagnal drzwi szafki do siebie"""
	x_relative = 0.55
	init_vector = PyKDL.Vector(x_relative, y_relative, z_relative) #wektor poczatkowy
	final_vector = cab_rot*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	gripper_rot = PyKDL.Rotation.RPY(0,0,Y-math.pi-0.2)	#obrot chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	print "UTWORZONO: macierz jednorodna dla chwytaka by pociagnal drzwi szafki"

	"""Pociagniecie drzwiczek"""
	lk=800
	ak=50
	imped = makeWrench([lk,lk,lk],[ak,ak,ak]),
	[x_g2,y_g2,z_g]=impedStearing(T_B_Trd,imped,0.05)
	[x_cos,y_cos,z_cos,theta_cos,Y_cos] = findObject("Tr")
	print "WYKONANO: pociagniecie drzwiczek punkt2: ", x_g2, y_g2
 	print "polozenie rzeczywiste" , x_cos, y_cos
	rospy.sleep(0.5)
	


	#POPRZEDNIE LICZENIE---------------------------------------------
	"""pobranie punktu nr 2 i wyliczenie R
	[x_p2, y_p2, fi] = getGripperXYfi()
	tnafi = math.tan(fi)
	temp = D2/math.sqrt(1+tnafi*tnafi)
	x_p1 = (abs(x_p1)+temp)*(abs(x_p1)/x_p1)
	y_p1 = (abs(y_p1)+temp*tnafi)*(abs(y_p1)/y_p1)
	x_len = abs(x_p1-x_p2)
	print x_len
	y_len = abs(y_p1-y_p2)
	print y_len
	R = math.sqrt(x_len*x_len+y_len*y_len)*0.5/math.cos(math.atan(x_len/y_len))
	print "Promien R =", R
	"""
	#POPRZEDNIE LICZENIE KONIEC---------------------------------------------
	


	"""Liczenie promienia okregu"""
	p1_coords = PyKDL.Vector(x_g1,y_g1,z_g)
	p1_rel_coords = cab_rot.Inverse()*(p1_coords-coords_cabinet)
	print "punkt 1 wzg szafki: ", p1_rel_coords
	p2_coords = PyKDL.Vector(x_g2,y_g2,z_g)
	p2_rel_coords = cab_rot.Inverse()*(p2_coords-coords_cabinet)
	print "punkt 2 wzg szafki: ", p2_rel_coords
	x_1 = abs(p1_rel_coords[0]-p2_rel_coords[0])
	y_1 = abs(p1_rel_coords[1]-p2_rel_coords[1])
	R = (x_1*x_1 + y_1*y_1)/(2*y_1)
	print "Promien R =", R

	#DODANY RUCH PO OKREGU----------------------------------------------------------------------------------------------------------------------
	"""Wyliczenie srodka okregu"""
	#x_p1, y_p1 - pierwszy punkt okregu
	#x_p2, y_p2 - drugi punkt okregu
	#p1_coords - vector dla 1 punktu okregu wzgledem robota
	#p1_rel_coords - vector dla 1 punktu okregu wzgledem szafki
	#center_rel_coords - wspolrzedne srodka okregu wzgledem szafki

	print "Wspolrzedne zadane", final_vector[0], final_vector[1]

	print "Wsporzedne Tr", x_g,y_g

	#print "Wspolrzedne Gr",x_p2, y_p2 

	#p1_coords = PyKDL.Vector(x_g1,y_g1,z_g)
	#p1_rel_coords = cab_rot.Inverse()*(p1_coords-coords_cabinet)
	center_rel_coords = PyKDL.Vector(p1_rel_coords[0], p1_rel_coords[1]+R, p1_rel_coords[2]) #wspolrzedne x srdoka okregu wzgledem szafki
	print "srodek okregu uklad szafki", center_rel_coords
	center_coords = cab_rot*center_rel_coords+coords_cabinet #wspolrzedne srodka okregu wzgledem robota
	print "Srodek okregu uklad robota:", center_coords
	rospy.sleep(2.5)


	"""Poruszanie chwytakiem po polokregu"""
	approx = 0.05 #odcinek, ktory aproxymuje bardzo maly wycinek okregu

	#p2_coords = PyKDL.Vector(x_g2,y_g2,final_vector[2]) #wspolrzedne punktu 2 wzgledem robota
	#p2_rel_coords = cab_rot.Inverse()*(p2_coords-coords_cabinet) #wspolrzedne punktu 2 wzgledem szafki
	p3_rel_coords = PyKDL.Vector(p2_rel_coords[0],p2_rel_coords[1],p2_rel_coords[2]) #wspolrzende kolejnego punktu okregu
	#ruszaj dopoki nie osiagnie wspolrzednej y takiej jak srodek okregu
	while p3_rel_coords[1] < center_rel_coords[1]:
	
		p3_rel_coords[1] += 0.05 #nowa wspolrzedna y dla chwytaka
		#sprawdzenie czy wartosci teoretycznie poprawane 
		if R*R-(center_rel_coords[1]-p3_rel_coords[1])*(center_rel_coords[1]-p3_rel_coords[1]) < 0 :
			print "R :", R
			print "p3_rel_coords[1]", p3_rel_coords[1]
			print "center_rel_coords[1]-p3_rel_coords[1]", (center_rel_coords[1]-p3_rel_coords[1])
			print "Wynik", R*R-(center_rel_coords[1]-p3_rel_coords[1])*(center_rel_coords[1]-p3_rel_coords[1])
			print "Wrong circle radius or goal point coordinates!--------------------------------------------------------------------"
			break
	

		p3_rel_coords[0] = math.sqrt(R*R-(center_rel_coords[1]-p3_rel_coords[1])*(center_rel_coords[1]-p3_rel_coords[1])) #wyliczenie wspolrzednej x dla kolejnego punktu okregu
		print "Uklad szafki:"
		print p3_rel_coords
		final_vector = cab_rot*p3_rel_coords+coords_cabinet #wektor przemieszczenia dla chwytaka
		T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka
		print "KOLEJNY RUCH uklad robota"
		print final_vector
		[x_g,y_g,z_g]=impedStearing(T_B_Trd,imped,0.05)
		rospy.sleep(0.3)

	

	"""Utworzenie macierzy jednorodnej dla chwytaka by wyswobodzil lape"""
	#x_relative = 0.45
	#y_relative = 0.5
	x_relative = p3_rel_coords[0]
	y_relative = p3_rel_coords[1]+0.1
	init_vector = PyKDL.Vector(x_relative, y_relative, z_relative) #wektor poczatkowy
	final_vector = cab_rot*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	#gripper_rot = PyKDL.Rotation.RPY(0,0,Y-math.pi+1.0)	#obrot chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	print "UTWORZONO: macierz jednorodna dla chwytaka by wyswobodzil lape"

	"""Wyswobodzenie lapy"""
	[x_g,y_g,z_g]=impedStearing(T_B_Trd,imped,0.05)
	print "WYKONANO: wyswobodzenie lapy"
	rospy.sleep(0.5)
	

	"""Przejscie do trybu jnp_imp"""
	toJnp()
	print "Przejscie do trybu jnp_imp"


	"""Powrot do pozycji poczatkowej"""
	moveVelmaJoints(q_map_starting)
	print "WYKONANO: powrot do pozycji poczatkowej"


	exitError(0)
 
