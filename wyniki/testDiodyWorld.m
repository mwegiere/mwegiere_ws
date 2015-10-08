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

skala = 1:70

figure(1);
clf(1);
hold on

plot (skala, diody_obroty, '.r')
plot (skala, diody_przesuniecie, '.g')
plot (skala, diody_przesuniecie_x, '.b')
plot (skala, diody_przesuniecie_y, '.c')
plot (skala, diody_przesuniecie_z, '.m')
legend('obroty', 'przesuniecie', 'przesuniecie_x','przesuniecie_y', 'przesuniecie_z')
hold off
