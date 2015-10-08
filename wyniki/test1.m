fileID = fopen('szachownica_obroty','r');
formatSpec = '%f';
szachownica_obroty = fscanf(fileID,formatSpec);

fileID = fopen('szachownica_przesuniecie','r');
formatSpec = '%f';
szachownica_przesuniecie = fscanf(fileID,formatSpec);

fileID = fopen('szachownica_przesuniecie_x','r');
formatSpec = '%f';
szachownica_przesuniecie_x = fscanf(fileID,formatSpec);

fileID = fopen('szachownica_przesuniecie_y','r');
formatSpec = '%f';
szachownica_przesuniecie_y = fscanf(fileID,formatSpec);

fileID = fopen('szachownica_przesuniecie_z','r');
formatSpec = '%f';
szachownica_przesuniecie_z = fscanf(fileID,formatSpec);

skala = 1:70

figure(1);
clf(1);
hold on

plot (skala, szachownica_obroty, '*r')
plot (skala, szachownica_przesuniecie, '*g')
legend('obroty','przesuniecie')
hold off
