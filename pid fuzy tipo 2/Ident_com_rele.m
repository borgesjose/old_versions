function [Kp,Ti,Td,Le] = Ident_com_rele(r1,c0,Tamostra,nptos,d,eps),


for t=1:2,
   u(t) = -d;e(t) = 0;y(t) = 0;tempo(t) = t*Tamostra;
end;

% --- Experimentação com o relé
for t = 3:nptos,
    
   %y(t) = 1.5143*y(t-1) + 0.5506*y(t-2) + 0.0199*u(t-1)+ 0.0163*u(t-2);
   %y(t) = 0.8825*y(t-1) + 0.235*u(t-1);
   y(t)= r1*y(t-1)+c0*u(t-1);
   
   e(t) = -y(t);
   if ((abs(e(t)) >= eps) & (e(t)  >0))      u(t) =  d; end;
   if ((abs(e(t)) > eps) & (e(t) < 0))      u(t) = -d; end;
   if ((abs(e(t)) < eps) & (u(t-1) == d))   u(t) =  d; end;
   if ((abs(e(t)) < eps) & (u(t-1) == -d))  u(t) = -d; end;
   tempo(t) = t*Tamostra;
end;

% --- Calcula período
kont = 0;								
for t = 4:nptos,								
   if u(t) ~= u(t-1)
      kont = kont + 1;
      ch(kont) = t;
   end
end

Tu1 = (ch(7) - ch(6))*Tamostra;
Tu2 = (ch(8) - ch(7))*Tamostra;
Tu = Tu1 + Tu2 %Periodo critico;
omega = (2*pi)/(Tu);
aux1 = ch(5);aux2 = ch(7);
% --- Calcula valor de pico positivo
arm = eps;										
for t = aux1:aux2,
   if y(t) >= arm  arm = y(t); end;
end;

Au = arm;
% --- Calcula valor de pico negativo
arm = eps;										

for t = aux1:aux2,
   if y(t) <= arm  arm = y(t); end;
end;

Ad = arm;
a = (abs(Au) + abs(Ad))/2;
% --- Calcula ganho critico
Ku = (4*d)/(pi*sqrt(a^2 - eps^2))
% --- Constantes da planta
%Calculo do atraso de transporte da planta;
num = 0;
den = 0;
for j=(nptos/2):(nptos/2)+ceil(Tu/Tamostra),
    num = num + y(j);
    den = den + u(j);
end
K = abs(num/den);

ti = (Tu/2*pi)*(sqrt((Ku*K)^2 - 1));

Le = (Tu/2*pi)*(pi - atan(((2*pi)/Tu)*ti));

% % --- Gráfico da saída e entrada
% figure(1)
% rele = [u;y];
% plot(tempo,rele);

%******************Identifica a FT do processo na frequência*******
gw=-(pi*sqrt(a^2-eps^2))/(4*d); %PROCESSO: valor da ft pro relé.

rp=abs(gw); 
fip=atan(eps/sqrt(a^2-eps^2)); 

%fip=angle(gw);

%omega = 2.618;

%*********Especificações*************
fim=70;
rs=8*rp;
fis=pi*fim/180;

%*************Cálculo dos Parâmetros do Controlador***********
Kc=rs*cos(fis-fip)/rp     
aux1=sin(fis-fip)/cos(fis-fip);
aux2=sqrt(1+aux1^2);
aux3=aux1+aux2;
Td=aux3/(2*omega)
Ti=4*Td
Kp=Kc;


end
