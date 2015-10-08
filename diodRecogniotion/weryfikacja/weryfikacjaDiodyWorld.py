#!/usr/bin/env python
import numpy as np
import math
from itertools import izip
import transformations

def liczenieOdleglosci():
	global homogMatrix
	homogMatrix = np.array([[0,0,0,2],[0,3,4,4],[5,5,6,3],[0,0,0,1]])
	odlegloscPozycji = np.sqrt(pow(homogMatrix[0][3],2) + pow(homogMatrix[0][3],2) + pow(homogMatrix[0][3],2))
	print homogMatrix[0][3]

def czytanieMacierzyT01():
	homogVectorX = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]	
	homogVectorY = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
	licznikWierszyX = 0
	licznikWierszyY = 0
	licznikWierszy = 0
	licznikMacierzy = 0
	wynikQuaternion = []
	wynikPrzesuniecie = []

	diody = 1
	szachownica = 0
	if diody:
                plik1 = "tfCamWorld.txt"
		plik2 = "wyniki2.txt"
	
	if diody:
		diody_obroty = open("../../wyniki/diody_obroty", "wb")
		diody_przesuniecie = open("../../wyniki/diody_przesuniecie", "wb")
	 	diody_przesuniecie_x = open("../../wyniki/diody_przesuniecie_x", "wb")
		diody_przesuniecie_y = open("../../wyniki/diody_przesuniecie_y", "wb")
		diody_przesuniecie_z = open("../../wyniki/diody_przesuniecie_z", "wb")
	
	with open("/home/mwegiere/ws_irp6/mwegiere_ws/src/serwo/diodRecogniotion/weryfikacja/"+plik1) as textfile1, open("/home/mwegiere/ws_irp6/mwegiere_ws/src/serwo/diodRecogniotion/weryfikacja/"+plik2) as textfile2:	
		for x, y in izip(textfile1, textfile2):
			homogVectorX[licznikWierszyX%16] = float(x)
			homogVectorY[licznikWierszyY%16] = float(y)
			licznikWierszyX = licznikWierszyX + 1
			licznikWierszyY = licznikWierszyY + 1
			licznikWierszy = licznikWierszy + 1
			
			if (licznikWierszy%16 == 15 ):
				homogVectorX[15] = 1.0
				homogMatrixX = np.matrix([[homogVectorX[0],homogVectorX[1],homogVectorX[2],homogVectorX[3]],[homogVectorX[4],homogVectorX[5],homogVectorX[6],homogVectorX[7]],[homogVectorX[8],homogVectorX[9],homogVectorX[10],homogVectorX[11]],[homogVectorX[12],homogVectorX[13],homogVectorX[14],homogVectorX[15]]])
				homogVectorY[15] = 1.0
				homogMatrixY = np.matrix([[homogVectorY[0],homogVectorY[1],homogVectorY[2],homogVectorY[3]],[homogVectorY[4],homogVectorY[5],homogVectorY[6],homogVectorY[7]],[homogVectorY[8],homogVectorY[9],homogVectorY[10],homogVectorY[11]],[homogVectorY[12],homogVectorY[13],homogVectorY[14],homogVectorY[15]]])
				
				Taa = homogMatrixX*homogMatrixY
		
				odlegloscPozycji = np.sqrt(pow(Taa.item(0,3),2) + pow(Taa.item(1,3),2) + pow(Taa.item(2,3),2))
				if diody:
					diody_przesuniecie.write(str(odlegloscPozycji)+"\n")
					diody_przesuniecie_x.write(str(Taa.item(0,3))+"\n")
					diody_przesuniecie_y.write(str(Taa.item(1,3))+"\n")
					diody_przesuniecie_z.write(str(Taa.item(2,3))+"\n")
						
				wynikPrzesuniecie.append(odlegloscPozycji)
				quaternion = transformations.quaternion_from_matrix([[Taa.item(0,0),Taa.item(1,0),Taa.item(2,0)],[Taa.item(1,0),Taa.item(1,1),Taa.item(1,2)],[Taa.item(2,0),Taa.item(2,1),Taa.item(2,2)]])

				odlegloscOrientacji = np.sqrt(2*(1-np.linalg.norm(quaternion*quaternion)))
				wynikQuaternion.append(odlegloscOrientacji)

				if diody:
					diody_obroty.write(str(odlegloscOrientacji)+"\n")

				licznikMacierzy = licznikMacierzy + 1
		mediana = 0
		for item in wynikQuaternion:
			mediana = mediana + float(item)
			#print item
		mediana = mediana/licznikMacierzy

		odchylenie = 0
		for item in wynikQuaternion:
			odchylenie = odchylenie + pow((float(item) - mediana),2)
		odchylenie = odchylenie / 3
		#print "odchylenie standardowe dla obrotu:  "
		#print np.sqrt(odchylenie)

		mediana = 0
		for item in wynikPrzesuniecie:
			mediana = mediana + float(item)
                        #print item
		mediana = mediana/licznikMacierzy

		odchylenie = 0
		for item in wynikPrzesuniecie:
			odchylenie = odchylenie + pow((float(item) - mediana),2)
		odchylenie = odchylenie / 3
		#print "odchylenie standardowe dla przesuniecia:  "
		#print np.sqrt(odchylenie)

		if diody:
			diody_obroty.close()
			diody_przesuniecie.close()
	 		diody_przesuniecie_x.close()
			diody_przesuniecie_y.close()
			diody_przesuniecie_z.close()

if __name__ == '__main__':
	#liczenieOdleglosci()
	czytanieMacierzyT01()

