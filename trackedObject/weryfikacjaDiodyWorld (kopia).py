#!/usr/bin/env python
import numpy as np
import math
from itertools import izip
from tf import transformations

#T_AB - przekształcenie z ukladu A do B, czyli pozycja ukladu B w ukladzie A
#D - uklad diod
#C - uklad optyczny kamery
#W - uklad swiata

#Tq_AB - kwaternion przekształcenia z ukladu A do B
#Tt_AB - translacja przekształcenia z ukladu A do B

#czytanie z pliku przeksztalcenia T_DC
#czytanie z pliku przekształcenia T_CW

#q = quaternion_multiply(qx, qy)
#T = translation_matrix((1, 2, 3))

class weryfication(self):

   def __init__(self):
      self.plik1 = "T_DC"
      self.plik2 = "T_CW"
   
   def czytanieMacierzy(self):
      with open("/home/mwegiere/ws_irp6/mwegiere_ws/src/serwo/diodRecogniotion/weryfikacja/"+plik1) as textfile1, open("/home/mwegiere/ws_irp6/mwegiere_ws/src/serwo/diodRecogniotion/weryfikacja/"+plik2) as textfile2:
         for x, y in izip(textfile1, textfile2):
            
      



def main(args):

if __name__ == '__main__':
   main(sys.argv)

