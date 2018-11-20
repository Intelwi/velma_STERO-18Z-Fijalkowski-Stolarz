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
	print "Znaleziono szafke", coords_cabinet
	

	
	"""Znalezienie zawiasow szafki"""
	[x_rd,y_rd,z_rd,theta_rd,Y_rd] = findObject("right_door")
	coords_right_door = PyKDL.Vector(x_rd,y_rd,z+0.13) #wspolrzedne zawiasow prawych drzwiczek
	print "Znaleziono zawiasy szafki",coords_right_door


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
	y_relative = 0.02
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
	lk=400
	ak=300
	imped = makeWrench([lk,lk,lk],[ak,ak,ak]),
	[x_g,y_g,z_g]=impedStearing(T_B_Trd,imped,0.1) #zwraca aktualne polozenie chwytaka
	x_g_relative = (PyKDL.Rotation.RPY(0,0,-Y)*(PyKDL.Vector(x_g,y_g,z_g)-coords_cabinet)).x() # wzgledny wektor polozenia przy uderzeniu
	print "x_g_relative:", x_g_relative
	print "WYKONANO: test walniecia w szafke"
	rospy.sleep(0.5)


	"""Utworzenie macierzy jednorodnej dla chwytaka by lekko cofnal reke"""
	x_relative = x_g_relative - 0.065
	init_vector = PyKDL.Vector(x_relative, y_relative, z_relative) #wektor poczatkowy
	final_vector = cab_rot*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	print "UTWORZONO: macierz jednorodna dla chwytaka by lekko cofnal reke"


	"""Lekkie cofnancie reki"""
	moveCart(T_B_Trd)
	print "WYKONANO: lekkie cofnancie reki"
	rospy.sleep(0.5)
	

	"""Wyliczenie punktu do ktorego ma poruszyc reka zeby trafic w uchwyt (doswiadczalne y)"""
	y_relative=-0.03 #doswiadczalne y tak by chwytak uderzyl w uchwyt
	init_vector = PyKDL.Vector(x_relative, y_relative, z_relative)
	final_vector = cab_rot*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka do uderzenia w uchwyt
	print "UTWORZONO: macierz jednorodna dla chwytaka by uderzyl w uchwyt szafki"

	"""Dojechanie do uchwytu"""
	lk=100
	ak=300
	imped = makeWrench([lk,lk,lk],[ak,ak,ak]),
	[x_g,y_g,z_g]=impedStearing(T_B_Trd,imped,0.1) #zwraca aktualne polozenie chwytaka
	print "WYKONANO: dojechanie do uchwytu"
	rospy.sleep(0.5)


	#Ten punkt jest bardzo wazny
	"""pobranie punktu nr 1"""
	rospy.sleep(1)
	[x_p1, y_p1, fi] = getGripperXYfi()
	tnafi = math.tan(abs(fi))
	temp = D2/math.sqrt(1+tnafi*tnafi)
	x_p1 = (abs(x_p1)+temp)*(abs(x_p1)/x_p1)
	y_p1 = (abs(y_p1)+temp*tnafi)*(abs(y_p1)/y_p1)
	print "PUNKT 1: ", x_p1, y_p1, "-----------------------------------------------------------","\n"

	
	"""Pobranie pozycji nadgarstka"""
	[x_g1,y_g1,z,theta,Y1] = findObject("Tr")
	gripper_vector1 = PyKDL.Vector(x_g1, y_g1, z)
	print "Pobranie pozycji nadgarstka"


	#w sumie to to nie jest juz potrzebne bo nie aproksymuje promienia
	"""Utworzenie macierzy jednorodnej dla chwytaka by pociagnal drzwi szafki do siebie
	x_relative = 0.54
	init_vector = PyKDL.Vector(x_relative, y_relative, z_relative) #wektor poczatkowy
	final_vector = cab_rot*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	print "UTWORZONO: macierz jednorodna dla chwytaka by pociagnal drzwi szafki"

	Pociagniecie drzwiczek
	lk=400
	lk1=100
	ak=300
	imped = makeWrench([lk,lk1,lk],[ak,ak,ak]),
	[x_g2,y_g2,z_g]=impedStearing(T_B_Trd,imped,0.07)
	rospy.sleep(1)
	"""


	#Sory Grzesiek ale niepotrzebne	
	#[x_p2, y_p2, fi] = getGripperXYfi()
	#print "PUNKT 2: ", x_p2, y_p2, "-----------------------------------------------------------\n"
	"""pobranie punktu nr 2 i wyliczenie R	
	tnafi = math.tan(fi)
	temp = D2/math.sqrt(1+tnafi*tnafi)
	x_p1_1 = (abs(x_p1)+temp)*(abs(x_p1)/x_p1)
	y_p1_1 = (abs(y_p1)+temp*tnafi)*(abs(y_p1)/y_p1)
	x_len = abs(x_p1_1-x_p2)
	print x_len
	y_len = abs(y_p1_1-y_p2)
	print y_len
	R = math.sqrt(x_len*x_len+y_len*y_len)*0.5/math.cos(math.atan(x_len/y_len))
	print "Promien R =", R
	R=R*2
	"""


	"""Wyliczenie promienia szafki"""
	rel_coords_right_door = cab_rot.Inverse()*(coords_right_door-coords_cabinet) #wspolrzedne zawiasow szafki
	R = math.sqrt(rel_coords_right_door[0]*rel_coords_right_door[0]+rel_coords_right_door[1]*rel_coords_right_door[1])
	print "Wyliczono promien: ",R,"\n"
	print rel_coords_right_door, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!@@@@@@@@@@@@@@1!!!!!!!!!2@!!!!!!!"


	"""Wyliczenie srodka okregu"""
	#x_p1, y_p1 - pierwszy punkt okregu
	#x_p2, y_p2 - drugi punkt okregu
	#p1_coords - vector dla 1 punktu okregu wzgledem robota
	#p1_rel_coords - vector dla 1 punktu okregu wzgledem szafki
	#center_rel_coords - wspolrzedne srodka okregu wzgledem szafki

	#p2_coords = PyKDL.Vector(x_p2,y_p2,final_vector[2]) #wspolrzedne punktu 2 wzgledem robota
	#p2_rel_coords = cab_rot.Inverse()*(p2_coords-coords_cabinet) #wspolrzedne punktu 2 wzgledem szafki

	p1_coords = PyKDL.Vector(x_p1,y_p1,z_g) #wspolrzedne klamki (punktu gdzie zderzyl sie z klamka)
	p1_rel_coords = cab_rot.Inverse()*(p1_coords-coords_cabinet) #to co powyzej tylko wzgledem szafki
	center_rel_coords = rel_coords_right_door #przepisalem bo tak bylo latwiej
	#center_rel_coords = PyKDL.Vector(p1_rel_coords[0], p1_rel_coords[1]+R, p1_rel_coords[2]) #wspolrzedne x srdoka okregu wzgledem szafki
	print "srodek okregu uklad szafki", center_rel_coords
	center_coords = coords_right_door #przepisalem bo tak bylo latwiej
	#center_coords = cab_rot*center_rel_coords+coords_cabinet #wspolrzedne srodka okregu wzgledem robota
	print "Srodek okregu uklad robota:", center_coords
	print "Szafka:",coords_cabinet
	#rospy.sleep(2.5)


	"""Przeskalowanie promienia dla robota dla nadgarstka (bo jest inny dla narzedzia)"""
	x = x_g1-p1_coords[0]
	y = y_p1-center_coords[1]
	R1 = math.sqrt(x*x+y*y) #R1 jest uzywany do rownania parametrycznego okregu po jakim porusza sie nadgarstek
	print "Przeskalowanie promienia do ruchu"


	#rospy.sleep(2.5)
	#[x_g2,y_g2,z,theta,Y1] = findObject("Tr")


	#to bylo w pierwszej wersji z aproksymacja ze jak pociagne to ustawia sie (zmienia orientacje) do ruchu po okregu 
	"""#Ustawienie reki do ruchu po okregu
	g2_coords = PyKDL.Vector(x_g2,y_g2,final_vector[2]) 
	
	#Wyliczenie startowego kata do ruchu narzedzia
	x = abs(p2_rel_coords[0]-p1_rel_coords[0])
	alpha = math.asin(x/R1)
	"""
	"""
	gripper_rot1 = PyKDL.Rotation.RPY(0,0,alpha+Y-math.pi) #obrot chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot1, g2_coords) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	[x_g,y_g,z_g]=impedStearing(T_B_Trd,imped,0.05)
	print "Ustawienie reki do ruchu po okregu----------------------------------------------------------------------- \n \n"
	rospy.sleep(3)
	"""

	"""Wylicznie kata do ruchu nadgarstka wzgledem kata do ruchu narzedzia"""
	alpha=0 #kat otwarcia szafki

	#kat dodawany do alpha (zeby byl do ruchu po okregu nadgarstka o promieniu R1)
	beta = math.asin(0.27/R1)#0.27 stala odleglosc narzedzia od nadgarstka od Dawida xd

	gamma = 0 #kat o jaki musi byc rozwarty okrag nadgarstka by okrag narzedzia byl rozwarty o alpha
	


	"""Poruszanie chwytakiem po polokregu"""
	p3_rel_coords = cab_rot.Inverse()*(gripper_vector1-coords_cabinet) #przerobienie poczatkowej pozycji nadgarstka na uklad szafki
	#g2_rel_coords = cab_rot.Inverse()*(g2_coords-coords_cabinet) #wspolrzedne punktu 2 wzgledem szafki
	#p3_rel_coords = PyKDL.Vector(g2_rel_coords[0],g2_rel_coords[1],g2_rel_coords[2]) #wspolrzende kolejnego punktu okregu
	

	"""Ustawienie impedancji"""
	lk=400
	ak=300
	imped = makeWrench([lk,lk,lk],[ak,ak,ak]),


	#ruszaj dopoki nie osiagnie kata pi/2 takiej jak srodek okregu
	while alpha<=math.pi/2:
		alpha=alpha+2*math.pi/20
		gamma = beta+alpha
		
		#rownania parametryczne okregu nadgarstka (nie narzedzia!)
		p3_rel_coords[0] = center_rel_coords[0] + R1*math.sin(gamma)
		p3_rel_coords[1] = center_rel_coords[1] - R1*math.cos(gamma)
		
		print "Uklad szafki:"
		print p3_rel_coords
		final_vector = cab_rot*p3_rel_coords+coords_cabinet #przerobienie pozycji nadgarstka na pozycje wzgledem robota
		gripper_rot1 = PyKDL.Rotation.RPY(0,0,alpha+Y-math.pi) #obrot nadgarstka
		T_B_Trd = PyKDL.Frame(gripper_rot1, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka
		print "KOLEJNY RUCH uklad robota"
		print final_vector
		[x_g,y_g,z_g]=impedStearing(T_B_Trd,imped,0.05)
		

	

	"""Utworzenie macierzy jednorodnej dla chwytaka by wyswobodzil lape
	#x_relative = 0.45
	#y_relative = 0.5
	x_relative = p3_rel_coords[0]
	y_relative = p3_rel_coords[1]+0.1
	init_vector = PyKDL.Vector(x_relative, y_relative, z_relative) #wektor poczatkowy
	final_vector = cab_rot*init_vector+coords_cabinet #wektor przemieszczenia dla chwytaka
	#gripper_rot = PyKDL.Rotation.RPY(0,0,Y-math.pi+1.0)	#obrot chwytaka
	T_B_Trd = PyKDL.Frame(gripper_rot, final_vector) #tworzenie macierzy jednorodnej do ustawienia chwytaka
	print "UTWORZONO: macierz jednorodna dla chwytaka by wyswobodzil lape"
"""
	"""Wyswobodzenie lapy
	[x_g,y_g,z_g]=impedStearing(T_B_Trd,imped,0.05)
	print "WYKONANO: wyswobodzenie lapy"
	rospy.sleep(0.5)
"""	

	"""Przejscie do trybu jnp_imp"""
	toJnp()
	print "Przejscie do trybu jnp_imp"


	"""Powrot do pozycji poczatkowej"""
	moveVelmaJoints(q_map_starting)
	print "WYKONANO: powrot do pozycji poczatkowej"


	exitError(0)
 
