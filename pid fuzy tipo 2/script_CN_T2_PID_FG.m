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
Theta_m_max = 72;%72;

a = 1/0.2133;
b = 0.6667/0.2133;
c = 0.1067/0.2133;
Lp = 2;
 %    
 
 Tc = 0.1;

b1 = 0.0092;
b2 = 0.0580;

a1 = -0.7893;
a2 = -0.1407;

% figure;
% step(ftz,50)

% Aplicando o relé:
Tc = 0.1;
Tamostra = Tc;
nptos = 1600;
Kc = 1.7519;
Ti = 0.6913;
Td = 0.1728;
nptos = 1600;
%Aplicando o CN-PID-FG:

 turnpoint = floor(rand*nptos);
for i=1:nptos,
    if (i<=turnpoint)  ref(i)=1; end;
    if (i>turnpoint)   ref(i) = 2; end;
end ;
% for i=1:nptos,
%     if (i<=nptos/4)  ref(i)=10; end;
%     if (i>nptos/4)   ref(i) = 40; end;
%     if (i>nptos/2 & i<=3*nptos/4)  ref(i)=50; end;
%     if (i>3*nptos/4)   ref(i) = 20; end;
% end ;
clear y;
y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;


erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;

L=2;%Provavelmente o valor de limite das memberships functions


%%
  
gene = populacao{h,1}(1:12);
Param_input = gene;
Param_output = [0.7871,0.5871,0.0495,0.0295,0.05,0.00];


    for i=5:nptos,

         y(i)= -r1*y(i-1)-r2*y(i-2)+c0*u(i-2)+c1*u(i-3)+c2*u(i-4);
        %y(i) =-a1*y(i-1)-a2*y(i-2)+b1*u(i-1)+b2*u(i-2);%+ruido(i);
         erro(i)=ref(i)-y(i); %Erro

         rate(i)=(erro(i)-erro(i-1));%/Tc; %Rate of erro


         %Aqui na chamada da função é possivel escolher entre linear input(LI) e não linear input(NLI)
%          Am(i) = Inferencia_T2_S(erro(i),rate(i),L,Param,'LI');

          Am(i) = inferencia_ortodoxa_S(erro(i),rate(i),L,Param_input,Param_output,'LI');
         Ami = Am(i)*Am_max + Am_min*(1 - Am(i)); 


          %Controlador:

                alpha = (Kc/Ami)*(1+((Td/Ami)/Tamostra)+(Tamostra/(2*(Ti*Ami))));
                beta = -(Kc/Ami)*(1+2*((Td/Ami)/Tamostra)-(Tamostra/(2*(Ti*Ami))));
                gama = (Kc/Ami)*(Td/Ami)/Tamostra;

                u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2);

           tempo(i)=i*Tamostra;
            
     end ;

     
