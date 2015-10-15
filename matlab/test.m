T_DC_Array = zeros(4,4,70);
fileID = fopen('T_DC.txt','r');
formatSpec = '%f';
sizeA = [4 4];

arrayCounter = 1;

while (arrayCounter<71) 
    A = fscanf(fileID,formatSpec,sizeA);
    T_DC_Array(:,:,arrayCounter)= A';
    arrayCounter = arrayCounter + 1;
    
end
%arrayCounter
fclose(fileID);


T_CW_Array = zeros(4,4,70);
fileID = fopen('T_CW.txt','r');
formatSpec = '%f';
sizeA = [4 4];

arrayCounter = 1;

while (arrayCounter<71) 
    A = fscanf(fileID,formatSpec,sizeA);
    T_CW_Array(:,:,arrayCounter)= A';
    arrayCounter = arrayCounter + 1;
    
end
%arrayCounter
fclose(fileID);

diody_przesuniecie_x = zeros(1,70);
diody_przesuniecie_y = zeros(1,70);
diody_przesuniecie_z = zeros(1,70);
diody_kat_a = zeros(1,70);
diody_kat_b = zeros(1,70);
diody_kat_c = zeros(1,70);

T_DW_Array = zeros(4,4,70);

arrayCounter=1;
while (arrayCounter<71) 
    T_CW = T_CW_Array(:,:,arrayCounter);
    T_WC=(T_CW)^(-1);
    T_DC = T_DC_Array(:,:,arrayCounter);
    T_CD = (T_DC)^(-1);
    if (T_DC == eye(4,4))
        T_DW_Array(:,:,arrayCounter) = 0;
        diody_przesuniecie_x(1,arrayCounter) = 0;
        diody_przesuniecie_y(1,arrayCounter) = 0;
        diody_przesuniecie_z(1,arrayCounter) = 0;
        diody_kat_a(1,arrayCounter) = 0;
        diody_kat_b(1,arrayCounter) = 0;
        diody_kat_c(1,arrayCounter) = 0;
        arrayCounter = arrayCounter + 1;  
    else
        T_DW_Array(:,:,arrayCounter) = T_WC*T_DC;
        [a,b,c]=GetEulerAngles(T_DW_Array(:,:,arrayCounter));   
        diody_przesuniecie_x(1,arrayCounter) = T_DW_Array(1,4,arrayCounter);
        diody_przesuniecie_y(1,arrayCounter) = T_DW_Array(2,4,arrayCounter);
        diody_przesuniecie_z(1,arrayCounter) = T_DW_Array(3,4,arrayCounter);
        diody_kat_a(1,arrayCounter) = a;
        diody_kat_b(1,arrayCounter) = b;
        diody_kat_c(1,arrayCounter) = c;
        arrayCounter = arrayCounter + 1;  
    end

end
% T_DW_Array(:,:,1)
% arrayCounter = 1
% while (arrayCounter<71) 
%     diody_przesuniecie_x(:,arrayCounter)
%     arrayCounter = arrayCounter + 1;
% end
skala = 1:70

figure(1);

clf(1);
hold on
title('Stabilność algorytmu wykrywającego wzorzec')
xlabel('-2\pi < x < 2\pi') % x-axis label
ylabel('sine and cosine values') % y-axis label
plot (skala, diody_przesuniecie_x, '.b')
plot (skala, diody_przesuniecie_y, '.c')
plot (skala, diody_przesuniecie_z, '.m')
plot (skala, diody_kat_a, '*b')
plot (skala, diody_kat_b, '*c')
plot (skala, diody_kat_c, '*m')
legend('przesuniecie w osi x','przesuniecie w osi y', 'przesuniecie w osi z','diody_kat_a','diody_kat_b','diody_kat_c')
hold off
