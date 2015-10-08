fileID = fopen('diody_obroty','r');
formatSpec = '%f';
diody_obroty = fscanf(fileID,formatSpec);

fileID = fopen('diody_przesuniecie','r');
formatSpec = '%f';
diody_przesuniecie = fscanf(fileID,formatSpec);

fileID = fopen('diody_przesuniecie_x','r');
formatSpec = '%f';
diody_przesuniecie_x = fscanf(fileID,formatSpec);

fileID = fopen('diody_przesuniecie_y','r');
formatSpec = '%f';
diody_przesuniecie_y = fscanf(fileID,formatSpec);

fileID = fopen('diody_przesuniecie_z','r');
formatSpec = '%f';
diody_przesuniecie_z = fscanf(fileID,formatSpec);

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

plot (skala, diody_obroty, '.r')
plot (skala, szachownica_obroty, '*r')
plot (skala, diody_przesuniecie, '.g')
plot (skala, szachownica_przesuniecie, '*g')
%plot (skala, diody_przesuniecie_x, '.b')
%plot (skala, szachownica_przesuniecie_x, '*b')
%plot (skala, diody_przesuniecie_y, '.c')
%plot (skala, szachownica_przesuniecie_y, '*c')
%plot (skala, diody_przesuniecie_z, '.m')
%plot (skala, szachownica_przesuniecie_z, '*m')
%legend('DIODYobroty','obroty', 'DIODYprzesuniecie', 'przesuniecie', 'DIODYprzesuniecie x','przesuniecie_x', 'DIODYprzesuniecie_y','przesuniecie_y', 'DIODYprzesuniecie_z', 'przesuniecie_z')
legend('DIODYobroty','obroty', 'DIODYprzesuniecie', 'przesuniecie')
hold off
