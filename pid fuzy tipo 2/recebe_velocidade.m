function velocidade = recebe_velocidade
global SerPIC
flushinput(SerPIC);
fprintf(SerPIC,'%c','v');
ler = fscanf(SerPIC,'%s');
velocidade = str2double(ler);
flushoutput(SerPIC);
end