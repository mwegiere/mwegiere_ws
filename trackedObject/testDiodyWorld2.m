
fid = fopen('diody','r');

tline = fgets(fid);
while ischar(tline)
    disp(tline)
    tline = fgets(fid);
end

fclose(fid);
