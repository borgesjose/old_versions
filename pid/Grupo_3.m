

a = (82.86 - 38.57)/2;%amplitude da oscila��o do sinal.
d = (100-30)/2;%amplitude do rele.
e = 6; %e = a 6 � pra funcionar...
T = (2.1);% periodo critico...
w = 2*pi/T; %frequencia critica...
Na = (4*d)*((sqrt(a^2 - e^2)/a) + (1i*e/a))/(pi*a); %Estimativa do ganho cr�tico...
Ft = -(1/Na);
plot(Ft);