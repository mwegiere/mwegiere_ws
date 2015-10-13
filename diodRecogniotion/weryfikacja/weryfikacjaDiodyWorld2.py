#!/usr/bin/env python
import numpy as np
import math
from itertools import izip
from tf import transformations

#T_AB - przeksztalcenie z ukladu A do B, czyli pozycja ukladu B w ukladzie A
#D - uklad diod
#C - uklad optyczny kamery
#W - uklad swiata

#Tq_AB - kwaternion przeksztalcenia z ukladu A do B
#Tt_AB - translacja przeksztalcenia z ukladu A do B

def run():
	T_CW = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]	
	T_DC = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
	licznikWierszy_T_CW = 0
	licznikWierszy_T_DC = 0
	licznikWierszy = 0
	licznikMacierzy = 0
	wynikPrzesuniecie = []
        plik1 = "T_CW.txt"
        plik2 = "T_DC.txt"
	
	diody_przesuniecie_x = open("/home/mwegiere/ws_irp6/mwegiere_ws/src/serwo/diodRecogniotion/weryfikacja/diody_przesuniecie_x", "wb")
	diody_przesuniecie_y = open("/home/mwegiere/ws_irp6/mwegiere_ws/src/serwo/diodRecogniotion/weryfikacja/wyniki/diody_przesuniecie_y", "wb")
	diody_przesuniecie_z = open("/home/mwegiere/ws_irp6/mwegiere_ws/src/serwo/diodRecogniotion/weryfikacja/wyniki/diody_przesuniecie_z", "wb")
	
	with open("/home/mwegiere/ws_irp6/mwegiere_ws/src/serwo/diodRecogniotion/weryfikacja/"+plik1) as textfile1, open("/home/mwegiere/ws_irp6/mwegiere_ws/src/serwo/diodRecogniotion/weryfikacja/"+plik2) as textfile2:	
		for x, y in izip(textfile1, textfile2):
			Vector_T_CW[licznik_T_CW%16] = float(x)
			Vector_T_DC[licznik_T_DC%16] = float(y)
			licznik_T_CW = licznik_T_CW + 1
			licznik_T_DC = licznik_T_DC + 1
			licznik = licznik + 1
			
			if (licznik%16 == 15 ):
				Vector_T_CW[15] = 1.0
				T_CW = np.matrix([[Vector_T_CW[0],Vector_T_CW[1],Vector_T_CW[2],Vector_T_CW[3]],[Vector_T_CW[4],Vector_T_CW[5],Vector_T_CW[6],Vector_T_CW[7]],[Vector_T_CW[8],Vector_T_CW[9],Vector_T_CW[10],Vector_T_CW[11]],[Vector_T_CW[12],Vector_T_CW[13],Vector_T_CW[14],Vector_T_CW[15]]])

				Vector_T_DC[15] = 1.0
				T_DC = np.matrix([[Vector_T_DC[0],Vector_T_DC[1],Vector_T_DC[2],Vector_T_DC[3]],[Vector_T_DC[4],Vector_T_DC[5],Vector_T_DC[6],Vector_T_DC[7]],[Vector_T_DC[8],Vector_T_DC[9],Vector_T_DC[10],Vector_T_DC[11]],[Vector_T_DC[12],Vector_T_DC[13],Vector_T_DC[14],Vector_T_DC[15]]])
				
				#T_DW = T_CW*T_DC
	
				diody_przesuniecie_x.write(str(T_CW.item(0,3)+T_DC.item(0,3))+"\n")
				diody_przesuniecie_y.write(str(T_CW.item(1,3)+T_DC.item(1,3))+"\n")
				diody_przesuniecie_z.write(str(T_CW.item(2,3)+T_DC.item(2,3))+"\n")

				licznik = licznik + 1
	
		if diody:
	 		diody_przesuniecie_x.close()
			diody_przesuniecie_y.close()
			diody_przesuniecie_z.close()

if __name__ == '__main__':
	run()

