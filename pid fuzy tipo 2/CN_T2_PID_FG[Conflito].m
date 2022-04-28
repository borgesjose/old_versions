%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piauí                       %
% Campus Ministro Petronio Portela                    %
% Copyright 2018 -José Borges do Carmo Neto-          %
% @author José Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  -- Desenvolvimento de um controlador nebuloso de   %
%  tipo 2  aplicado a autosintonia de PID, baseado    % 
%  nas margens de fase e de ganho                     %
%  -- Version: 1.0  - 05/03/2019                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%
% Tratando o processo:            
%clear;
p1=(1/2.5);
p2=(1/3.75);
k=(2/(2.5*3.75));
Tc=0.2;
Tamostra = Tc;
%%

s = tf('s');

ft = k/((s+p1)*(s+p2));

ftz = c2d(ft,Tc,'zoh');

%step(ftz,50)

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

%% Definições do Controlador: 

Am_min = 2; 
Am_max = 5;
Theta_m_min = 30;%45;
Theta_m_max = 60;%72;

a = 1/0.2133;
b = 0.6667/0.2133;
c = 0.1067/0.2133;
Lp = 2;
     
%Aplicando o CN-PID-FG:

g0=Kp*(1+(Td/Tc)+(Tc/Ti));
g1=-Kp*(1+2*(Td/Tc));
g2=Kp*Td/Tc;


 turnpoint = 500;%floor(rand*nptos);
for i=1:nptos,
    if (i<=turnpoint)  ref(i)=1; end;
    if (i>turnpoint)   ref(i) = 2; end;
end ;

for i=1:nptos,
    if (i<=nptos/4)  ref(i)=2; end;
    if (i>nptos/4)   ref(i) = 1; end;
    if (i>nptos/2 & i<=3*nptos/4)  ref(i)=2; end;
    if (i>3*nptos/4)   ref(i) = 1; end;
end ;
clear y;
y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;

U_d(1)=0 ; U_d(2)=0  ;U_d(3)=0; U_d(4)=0; 
U_i(1)=0 ; U_i(2)=0  ; U_i(3)=0; U_i(4)=0; 
U_p(1)=0 ; U_p(2)=0  ; U_p(3)=0; U_p(4)=0;

erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;

L=2;%Provavelmente o valor de limite das memberships functions

Kd=2*Kp*Td/Tc; %Valor do ganho associado ao termo derivativo do PID sintonizado pelo relé;  
Ki=(Kp*Tc)/(2*Ti);
Kc = Kp;

%%
%randpmo criado aleatoriamente por mim
%gene =[0.5,0.5,0.5,0.5,0.5,0.51,.51,.41,0.41,0.41,0.31,0.31,0.41,0.51,.21,.11];

%Melhor resulatado encontrado pelo AG ate hoje, para referencia descendente:
melhor_gene =[0.1207 0.4157,0.0952,0.0245,0.2265,0.5705,0.5739,0.5749,0.4667,0.3058,0.1700,0.5387,0.4378,0.7284,0.5696,0.0467];

%Encontrado para uma referencia variavel
%melhor_gene2 = [0.0053,0.5896,0.3968,0.2545,0.4845,1.4901,0.4170,0.4718,0.6184,0.0319,0.5025,0.3438,0.4768,0.6034,0.2274,0.5482];
%gene = thebest{1:1}(1:16);

gene = melhor_gene; 
Param = [gene,.95];

    for i=5:nptos,
%          if (i==550),r1 = - 1.909;r2 = 0.9109;  end
%          if (i==150),r1 = - 1.88;r2 = 0.88;  end
         y(i)= -r1*y(i-1)-r2*y(i-2)+c0*u(i-2)+c1*u(i-3)+c2*u(i-4);

         erro(i)=ref(i)-y(i); %Erro

         rate(i)=(erro(i)-erro(i-1));%/Tc; %Rate of erro


         %Aqui na chamada da função é possivel escolher entre linear input(LI) e não linear input(NLI)
         Am(i) = Inferencia_T2(erro(i),rate(i),L,Param,'LI');
         %[Am(i),yl(i),yr(i),l(i),r(i)]=EIASC(R(:,1)',R(:,2)',Y(:,1)',Y(:,2)'); 

         Ami = Am(i)*Am_max + Am_min*(1 - Am(i)); 

          KuP = Kc/Ami;
          KuI = Ki/Ami;
          KuD = Kd/(Ami)^2;

          U_p(i)=  KuP; 
          U_d(i)= KuD;
          U_i(i)= KuI;

          %Controlador:

                alpha = (Kc/Ami)*(1+((Td/Ami)/Tamostra)+(Tamostra/(2*(Ti*Ami))));
                beta = -(Kc/Ami)*(1+2*((Td/Ami)/Tamostra)-(Tamostra/(2*(Ti*Ami))));
                gama = (Kc/Ami)*(Td/Ami)/Tamostra;

                u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2);

           tempo(i)=i*Tamostra;
            
          %fprintf('amostra:  %d \t entrada:  %6.3f \t saida:  %4.0f\n',i,u(i),y(i));

     end ;

     ISE_t2 = objfunc(erro,tempo,'ISE')
     ITSE_t2 = objfunc(erro,tempo,'ITSE')
     ITAE_t2 = objfunc(erro,tempo,'ITAE')
     IAE_t2 = objfunc(erro,tempo,'IAE')
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
%PLOTAGEM
%Relé
% figure;
% grid;
% plot(yr,'c-');
% hold on;
% plot(ur);

%Saida do processo:
%figure;
% grid;
plot(tempo,y,'b-');
hold on;
%plot(tempo,u);

%title(['T2,IM-1: ISE:', num2str(ISE_t2), ', ITSE:' ,num2str(ITSE_t2),', IAE:' ,num2str(IAE_t2), ', ITAE:' ,num2str(ITAE_t2)])
title('Resposta ao degrau com mudança referência')
legend('Type-1-CN-PID-FG','Type-2-CN-PID-FG')
plot(tempo,ref,'r--');
xlabel('tempo(s)');
ylabel('y(t)');
% text1= {'T1: ISE:', num2str(ISE_t1), ', ITSE:' ,num2str(ITSE_t1),', IAE:' ,num2str(IAE_t1), ', ITAE:' ,num2str(ITAE_t1)}
% text2= {'T2,IM-1: ISE:', num2str(ISE_t2), ', ITSE:' ,num2str(ITSE_t2),', IAE:' ,num2str(IAE_t2), ', ITAE:' ,num2str(ITAE_t2)}
% text([100],[0.5],text2)
