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

T_DW_Array = zeros(4,4,70);

arrayCounter=1;
while (arrayCounter<71) 
    T_CW = T_CW_Array(:,:,arrayCounter);
    T_DC = T_DC_Array(:,:,arrayCounter);
    %T_DW_Array(:,:,arrayCounter)= T_CW*T_DC; zle
    T_DW_Array(:,:,arrayCounter)= (T_CW)^(-1)*T_DC; %OK
    %T_DW_Array(:,:,arrayCounter)= T_CW*(T_DC)^(-1);zle
     %T_DW_Array(:,:,arrayCounter)= (T_CW)^(-1)*(T_DC)^(-1);tak sobie


    
    diody_przesuniecie_x(1,arrayCounter) = T_DW_Array(1,4,arrayCounter);
    diody_przesuniecie_y(1,arrayCounter) = T_DW_Array(2,4,arrayCounter);
    diody_przesuniecie_z(1,arrayCounter) = T_DW_Array(3,4,arrayCounter);
    arrayCounter = arrayCounter + 1;  
    

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

plot (skala, diody_przesuniecie_x, '.b')
plot (skala, diody_przesuniecie_y, '.c')
plot (skala, diody_przesuniecie_z, '.m')
legend('przesuniecie_x','przesuniecie_y', 'przesuniecie_z')
hold off
