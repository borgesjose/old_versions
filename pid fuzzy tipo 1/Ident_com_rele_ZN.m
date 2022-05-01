
function [Kp,Ti,Td] = Ident_com_rele_ZN(Tamostra,nptos,d,eps),
b0=1.9892;
b1=1.3949;
b2=0.1939;
a0=1;
a1=-1.1003;
a2=0.2423;
a3= -0.0532;
a4= 0.0409;
y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;
for t=1:2,
   u(t) = -d;e(t) = 0;y(t) = 0;tempo(t) = t*Tamostra;
end;
r1 = 0.929;
c0 = 0.01893;
% --- Experimentação com o relé
for t = 5:nptos,								
y(t)= -a1*y(t-1)-a2*y(t-2)-a3*y(t-3)-a4*y(t-4)+b0*u(t-2)+b1*u(t-3)+b2*u(t-4);
   e(t) = -y(t);
   if ((abs(e(t)) >= eps) & (e(t)  >0))      u(t) =  d; end;
   if ((abs(e(t)) > eps) & (e(t) < 0))      u(t) = -d; end;
   if ((abs(e(t)) < eps) & (u(t-1) == d))   u(t) =  d; end;
   if ((abs(e(t)) < eps) & (u(t-1) == -d))  u(t) = -d; end;
   tempo(t) = t*Tamostra;
end;
%%
% --- Calcula período
kont = 0;								
for t = 4:nptos,								
   if u(t) ~= u(t-1)
      kont = kont + 1;
      ch(kont) = t;
   end
end
%%
Tu1 = (ch(7) - ch(6))*Tamostra;
Tu2 = (ch(8) - ch(7))*Tamostra;
Tu = Tu1 + Tu2; %Periodo critico;
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
Ku = (4*d)/(pi*sqrt(a^2 - eps^2));
% --- Constantes da planta

% %% --- Gráfico da saída e entrada
% figure(1)
% rele = [u;y];
% plot(tempo,rele);
%% --- Sintonia por Ziegler-Nichols
Kc = 0.6*Ku;
Ti = 0.5*Tu;
Td = Ti/4;
Kp = Kc;






end 