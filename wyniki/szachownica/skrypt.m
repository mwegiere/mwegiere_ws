fileID = fopen('obroty.txt','r');
formatSpec = '%f';
obroty = fscanf(fileID,formatSpec)

fileID = fopen('przesuniecie.txt','r');
formatSpec = '%f';
przesuniecie = fscanf(fileID,formatSpec);

fileID = fopen('przesuniecie_x.txt','r');
formatSpec = '%f';
przesuniecie_x = fscanf(fileID,formatSpec);

fileID = fopen('przesuniecie_y.txt','r');
formatSpec = '%f';
przesuniecie_y = fscanf(fileID,formatSpec);

fileID = fopen('przesuniecie_z.txt','r');
formatSpec = '%f';
przesuniecie_z = fscanf(fileID,formatSpec);

skala = [0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 18 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69];

plot [skala, obroty, 'r', skala, przesuniecie, 'g', skala, przesuniecie_x, 'b',skala, przesuniecie_y, 'c',skala, przesuniecie_z, 'm']
