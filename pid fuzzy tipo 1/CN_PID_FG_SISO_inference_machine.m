% Tratando o processo:            
p1=(1/2.5);
p2=(1/3.75);
k=(2/(2.5*3.75));
Tc=0.2;
Tamostra = Tc;
%%

s = tf('s')

ft = k/((s+p1)*(s+p2))

ftz = c2d(ft,Tc,'zoh');
figure;
step(ftz,100)
stepinfo(ftz,100)
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

%% Aplicando o relé:

n=200;
dh=0.5;   eps=0.2;
nptos=1000; 


[yr,ur]=proc(n,p1,p2,k,Tc,dh,eps);

tempo_= Tc*(1:1:size(yr,2))
ur = ur(1:200);
figure;
grid;
plot(tempo_,yr,'c-');
hold on;
plot(tempo_,ur);

%% Identificando pelo metodo do relé:
[gw,w,arm]=ident1(n, dh, eps,Tc,yr);

rs=1;
fim=60;
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

Am_min = 2; 
Am_max = 5;
Theta_m_min = 45;
Theta_m_max = 72;

a = 1/0.2133;
b = 0.6667/0.2133;
c = 0.1067/0.2133;
Lp = 2;

%     %Margem de fase:
%%
%         Theta_m = (pi/2)*(1-(1/Am));
% 
%         [Kc; Ki; Kd] = (pi/(2*Am(i)*Lp))*[b;c;a];

%% Aplicando o CN-PID-FG:

g0=Kp*(1+(Td/Tc)+(Tc/Ti));
g1=-Kp*(1+2*(Td/Tc));
g2=Kp*Td/Tc;

 turnpoint = 500;%floor(rand*nptos);
for i=1:nptos,
    if (i<=turnpoint)  ref(i)=1; end;
    if (i>turnpoint)   ref(i) =2 ; end;
end ;

y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
% for i=1:nptos,
%     if (i<=nptos/4)  ref(i)=1; end;
%     if (i>nptos/4)   ref(i) = 3; end;
%     if (i>nptos/2 & i<=3*nptos/4)  ref(i)=2; end;
%     if (i>3*nptos/4)   ref(i) = 1; end;
%  end ;

% for i=1:nptos,
%     
%     ref(i)=1;
%     
%  end ;


u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;

erro(1)=0 ; erro(2)=0 ; erro(3)=0; erro(4)=0;

L=2;%Provavelmente o valor de limite das memberships functions

Kd=2*Kp*Td/Tc; %Valor do ganho associado ao termo derivativo do PID sintonizado pelo relé;  
Ki=(Kp*Tc)/(2*Ti);
Kc = Kp;
rlevel = 0.1;
%ruido = rlevel*rand(1,1000);
for i=5:nptos,
 %ruin=(0.5-rand);
 %l(i) = 2 + ruin;%(0.5-rand);
L=2;%l(i);  
p1=(1/2.5);%+rlevel*rand;
 %if (i==500),p1 =  1.5/2.5;  end
p2= (1/3.75);
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
% Ti=1.424;
% Td = 1.6327;
% Kc = 5.6962;

%     if (i==150),r1 = - 1.88;r2 = 0.88;  end
ru(i) = 0.00*rand;
     y(i)= -r1*y(i-1)-r2*y(i-2)+c0*u(i-2)+c1*u(i-3)+c2*u(i-4) + ru(i);
     %temp = recebe_temperatura
     %y(i)=temp(3);
     erro(i)=ref(i)-y(i);%+ruido(i); %Erro
     
     rate(i)=(erro(i)-erro(i-1));%/Tc; %Rate of erro
     
     Am(i)=Inferencia(erro(i),rate(i),L,'LI');
     %Am(i)=inferencia_LO(erro(i),rate(i),L,'LI');
     %Aqui na chamada da função é possivel escolher entre linear input(LI) e não linear input(NLI)            
     %Am(i)=1;
     Ami = Am(i)*Am_max + Am_min*(1 - Am(i)); 
     Ami = 1;
%       Kaux = (pi/(2*Ami*Lp))*[b;c;a];
%       
%       Kc = Kaux(1);
%       Ki = Kaux(2);
%       Kd = Kaux(3);

      %Controlador:

            alpha = (Kc/Ami)*(1+((Td/Ami)/Tamostra)+(Tamostra/(2*(Ti*Ami))));
            beta = -(Kc/Ami)*(1+2*((Td/Ami)/Tamostra)-(Tamostra/(2*(Ti*Ami))));
            gama = (Kc/Ami)*(Td/Ami)/Tamostra;
%             alpha = (Kc/Ami)*(1+((Td)/Tamostra)+(Tamostra/(2*(Ti))));
%             beta = -(Kc/Ami)*(1+2*((Td)/Tamostra)-(Tamostra/(2*(Ti))));
%             gama = (Kc/Ami)*(Td)/Tamostra;
            %u(i)= erro(i);
            u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2);
            %if (u(i)<0) u(i) = 0; end;
            %u(i)= u(i-1)+U_p(i)+U_i(i)+U_d(i);
       tempo(i)=i*Tamostra;
       
      fprintf('amostra:  %d \t entrada:  %6.3f \t saida:  %4.0f\n',i,u(i),y(i));
      
 end ;
 %%
     H = nptos;
     It1 = esforco_ponderado(erro,u,H,0.5);
     ISE_t1 = objfunc(erro,tempo,'ISE')
     ITSE_t1 = objfunc(erro,tempo,'ITSE')
     ITAE_t1 = objfunc(erro,tempo,'ITAE')
     IAE_t1 = objfunc(erro,tempo,'IAE')
     
     
%%
figure;
%grid;
plot(tempo,y,'m-');
hold on;
plot(tempo,ref);
figure;
hold on;
plot(tempo,u,'c-');


% title(['T1: ',num2str(rlevel), ' : ISE:', num2str(ISE_t1), ', ITSE:' ,num2str(ITSE_t1),', IAE:' ,num2str(IAE_t1), ', ITAE:' ,num2str(ITAE_t1)])
%plot(tempo,Am,'m-')
%It1
%%
sy_t1= var(y);
su_t1 = var(u);


