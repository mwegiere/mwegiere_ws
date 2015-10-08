fileID = fopen('diody_obroty','r');
formatSpec = '%f';
diody_obroty = fscanf(fileID,formatSpec);

fileID = fopen('diody_przesuniecie','r');
formatSpec = '%f';
diody_przesuniecie = fscanf(fileID,formatSpec);

skala = 1:70

figure(1);
clf(1);
hold on

plot (skala, diody_obroty, '*r')
plot (skala, diody_przesuniecie, '*g')
legend('obroty','przesuniecie')
hold off
