% Tratando o processo:            
clear;
p1=(1/2.5);
p2=(1/3.75);
k=(2/(2.5*3.75));
Tc=0.2;
Tamostra = Tc;
%%
a0=k/(p1*p2);
a1=k/(-p1*(p2-p1));
a2=k/(-p2*(p1-p2));
x1=-exp(-p1*Tc);
x2=-exp(-p2*Tc);
x3=x1+x2;
x4=exp(-(p1+p2)*Tc);

%%
c0=a0+a1+a2;

c1=a0*x3+a1*(x2-1)+a2*(x1-1);

c2=a0*x4-a1*x2-a2*x1;

r0=1;
r1=x3;
r2=x4;
%%

s = tf('s')

ft = k/((s+p1)*(s+p2))

ftz = c2d(ft,Tc,'zoh')

%step(ftz,50)
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      

%% Aplicando o relé:

n=200;
dh=0.5;   eps=0.2;
nptos=1000; 

[yr,ur]=proc(n,p1,p2,k,Tc,dh,eps);

%%
% figure;
% grid;
% plot(yr,'c-');
% hold on;
% plot(ur);

%% Identificando pelo metodo do relé:
[gw,w,arm]=ident1(n, dh, eps,Tc,yr);

rs=1;
fim=45;
rp=pi*arm/(4*dh);

fip=atan(eps/sqrt(arm^2-eps^2));

fis=pi*fim/180;

%PID pelo metodo de Astron e Hangglund:

Kp=rs*cos(fis-fip)/rp;
         
aux1=sin(fis-fip)/cos(fis-fip);
aux2=sqrt(1+aux1^2);
aux3=aux1+aux2;

Ti=aux3/(2*w);
Td=4*Ti;
%% Definições de 

Am = 3;

Am_min = 2; 
Am_max = 5;
Theta_m_min = 45;
Theta_m_max = 72;

a = 1;
b = 0.6667;
c = 0.1067;
Lp = 2;

%     %Margem de fase:
%%
Theta_m = (pi/2)*(1-(1/Am));
% 
% [Kc; Ki; Kd] = (pi/(2*Am*Lp))*[b;c;a];

%% Aplicando o CN-PID-FG:

g0=Kp*(1+(Td/Tc)+(Tc/Ti));
g1=-Kp*(1+2*(Td/Tc));
g2=Kp*Td/Tc;
%  
for i=1:nptos,
    if (i<=nptos/2)  ref(i)=1; end;
    if (i>nptos/2)   ref(i) = 2; end;
end ;

% for i=1:nptos,
%     if (i<=nptos/4)  ref(i)=2; end;
%     if (i>nptos/4)   ref(i) = 1; end;
%     if (i>nptos/2 & i<=3*nptos/4)  ref(i)=2; end;
%     if (i>3*nptos/4)   ref(i) = 1; end;
%   end ;

y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;

erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;

L=2;%Provavelmente o valor de limite das memberships functions

% Kd=2*Kp*Td/Tc; %Valor do ganho associado ao termo derivativo do PID sintonizado pelo relé;  
% Ki=(Kp*Tc)/(2*Ti);
% Kc = Kp;
%%
        K = (pi/(2*Am*Lp))*[b;c;a];
        Kc = K(1);
        Ki = K(2);
        Kd = K(3);
%%
rlevel = 0.0;
ruido = rlevel*rand(1,1000);

for i=5:nptos,

p1=(1/2.5)+rlevel*rand;
p2= (1/3.75)+ruido(i);
k=(2/(2.5*3.75));

a0=k/(p1*p2);
a1=k/(-p1*(p2-p1));
a2=k/(-p2*(p1-p2));
x1=-exp(-p1*Tc);
x2=-exp(-p2*Tc);
x3=x1+x2;
x4=exp(-(p1+p2)*Tc);

c0=a0+a1+a2;
c1=a0*x3+a1*(x2-1)+a2*(x1-1);
c2=a0*x4-a1*x2-a2*x1;
r0=1;
r1=x3;
r2=x4; 
     if (i==550),r1 = - 1.84;r2 = 0.9109;  end
     y(i)= -r1*y(i-1)-r2*y(i-2)+c0*u(i-2)+c1*u(i-3)+c2*u(i-4);
     
     erro(i)=ref(i)-y(i); %Erro
      
      %Controlador:

            alpha = (Kc)*(1+((Td)/Tamostra)+(Tamostra/(2*(Ti))));
            beta = -(Kc)*(1+2*((Td)/Tamostra)-(Tamostra/(2*(Ti))));
            gama = (Kc)*(Td)/Tamostra;
        
            u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2);
      
       tempo(i)=i*Tamostra;
      fprintf('amostra:  %d \t entrada:  %6.3f \t saida:  %4.0f\n',i,u(i),y(i));
      
 end ;
 
 
      ISE_t2 = objfunc(erro,tempo,'ISE')
     ITSE_t2 = objfunc(erro,tempo,'ITSE')
     ITAE_t2 = objfunc(erro,tempo,'ITAE')
     IAE_t2 = objfunc(erro,tempo,'IAE')
figure;
grid;
plot(tempo,y,'g-');
hold on;
plot(tempo,u);
plot(tempo,ref);
title(['Astrom:',num2str(rlevel), ' ISE:', num2str(ISE_t2), ', ITSE:' ,num2str(ITSE_t2),', IAE:' ,num2str(IAE_t2), ', ITAE:' ,num2str(ITAE_t2)])

     
     
     