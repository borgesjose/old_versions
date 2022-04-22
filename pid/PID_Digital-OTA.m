%##################Roda o Algotítmo PID Discreto/Digital ################

%Lê Kc, Ti e Td do prompt do MATLAB
%Lê Tamostra do Prompt

varlist = {'r','u','y', 'tempo'};
clear(varlist{:})
clf(figure(2))

nptos=500; 

g0=Kc*(1+(Td/Tamostra)+(Tamostra/Ti));
g1=-Kc*(1+2*(Td/Tamostra));
g2=Kc*Td/Tamostra;

alpha = Kc*(1+(Tamostra/(2*Ti))) ;%Kc*(1+(Td/Tamostra)+(Tamostra/(2*Ti))) ;
beta = -Kc*(1 -(Tamostra/(2*Ti)));%-Kc*(1+2*(Td/Tamostra)-(Tamostra/(2*Ti)));
gama = 0;%Kc*Td/Tamostra;

for i=1:nptos,
    if (i<=50)  r(i)=1; end;
    if (i>50)   r(i)=2; end;
    if (i>150)  r(i)=3; end;
    if (i>250)   r(i)=2; end;
    if (i>350)   r(i)=1; end;
 end ;

y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;

erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;
Kd=2*Kc*Td/Tamostra;
Ki=(Kc*Tamostra)/(2*Ti);
for t=3:nptos,
     %y(t) = 1.5143*y(t-1) - 0.5506*y(t-2) + 0.0199*u(t-1)+ 0.0163*u(t-2);
     y(t) = 0.8825*y(t-1) + 0.235*u(t-1);
     erro(t)=r(t)-y(t);
     
      %u(t)= u(t-1) + g0*erro(t) + g1*erro(t-1) + g2*erro(t-2);
      %u(t)= u(t-1) + Kc*(Tamostra/Ti)*r(t) - g0*y(t) - g1*y(t-1) - g2*y(t-2);
      u(t)= u(t-1) + alpha*erro(t) + beta*erro(t-1) + gama*erro(t-2);
      tempo(t)=t*Tamostra;
        
   end ;
   
figure(2)

plot(tempo,r,'r','LineWidth',2);
hold on;
plot(tempo,u,'b','LineWidth',2);
hold on;
plot(tempo,y,'g','LineWidth',2);
hold on;

drawnow
grid on;
