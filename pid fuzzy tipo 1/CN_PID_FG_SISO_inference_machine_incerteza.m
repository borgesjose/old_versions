% Tratando o processo:            
Tc = 5;
Tamostra = Tc;
n=200;
d=50;   eps=1;
nptos=1000; 

[Kp,Ti,Td] = Ident_com_rele(Tamostra,nptos,d,eps);
[Kp,Ti,Td] = Ident_com_rele_ZN(Tamostra,nptos,d,eps);
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
    if (i<=turnpoint)  ref(i)=4; end;
    if (i>turnpoint)   ref(i) =1 ; end;
end ;

y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
for i=1:nptos,
    if (i<=nptos/4)  ref(i)=300; end;
    if (i>nptos/4)   ref(i) = 100; end;
    if (i>nptos/2 & i<=3*nptos/4)  ref(i)=500; end;
    if (i>3*nptos/4)   ref(i) = 200; end;
 end ;

u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;
u(5)=0 ; u(6)=0 ; u(3)=0; u(4)=0;
erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;
erro(5)=1 ; erro(6)=1 ;
L=2;%Provavelmente o valor de limite das memberships functions

Kd=2*Kp*Td/Tc; %Valor do ganho associado ao termo derivativo do PID sintonizado pelo relé;  
Ki=(Kp*Tc)/(2*Ti);
Kc = Kp;
rlevel = 0.2;

B0 = [1.9252:0.0001:2.0472]
B1 = [0.5757:0.0001:2.2193]
B2 = [-0.8782:0.0001:0.5073]

A1 = [-1.4956:0.0001:-0.6924]
A2 = [-0.14956:0.0001:0.6117]
A3 = [-0.1235:0.0001:0.0023]
A4 = [0.0045:0.0001:0.0850]

for i=7:nptos,
    
b0= B0(randi([size(B0)]));
b1=B1(randi([size(B1)]));
b2=B2(randi([size(B2)]));
a0=1;
a1=A1(randi([size(A1)]));
a2=A2(randi([size(A2)]));
a3= A3(randi([size(A3)]));
a4= A4(randi([size(A4)]));

b0=1.9892;
b1=1.3949;
b2=0.1939;
a0=1;
a1=-1.1003;
a2=0.2423;
a3= -0.0532;
a4= 0.0409;
         y(i)= -a1*y(i-1)-a2*y(i-2)-a3*y(i-3)-a4*y(i-4)+b0*u(i-4)+b1*u(i-5)+b2*u(i-6);
     erro(i)=ref(i)-y(i); %Erro
     
     rate(i)=(erro(i)-erro(i-1));%/Tc; %Rate of erro
     
     Am(i)=Inferencia(erro(i),rate(i),L,'LI');
     %Am(i)=inferencia_LO(erro(i),rate(i),L,'LI');
     %Aqui na chamada da função é possivel escolher entre linear input(LI) e não linear input(NLI)            
     %Am(i)=1;
      Ami = Am(i)*Am_max + Am_min*(1 - Am(i)); 
      
%       Kaux = (pi/(2*Ami*Lp))*[b;c;a];
%       
%       Kc = Kaux(1);
%       Ki = Kaux(2);
%       Kd = Kaux(3);
      
      KuP = Kc/Ami;
      KuI = Ki/Ami;
      KuD = Kd/Ami;
      
      U_p(i)=  KuP; 
      U_d(i)= KuD;
      U_i(i)= KuI;
      
      %Controlador:

            alpha = (Kc/Ami)*(1+((Td/Ami)/Tamostra)+(Tamostra/(2*(Ti*Ami))));
            beta = -(Kc/Ami)*(1+2*((Td/Ami)/Tamostra)-(Tamostra/(2*(Ti*Ami))));
            gama = (Kc/Ami)*(Td/Ami)/Tamostra;
        
            u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2);
      
            %u(i)= u(i-1)+U_p(i)+U_i(i)+U_d(i);
       tempo(i)=i*Tamostra;
       
      fprintf('amostra:  %d \t entrada:  %6.3f \t saida:  %4.0f\n',i,u(i),y(i));
      
 end ;
     ISE_t1 = objfunc(erro,tempo,'ISE')
     ITSE_t1 = objfunc(erro,tempo,'ITSE')
     ITAE_t1 = objfunc(erro,tempo,'ITAE')
     IAE_t1 = objfunc(erro,tempo,'IAE')
%figure;
grid;
plot(tempo,y,'c-');
hold on;
plot(tempo,u,'g-');
plot(tempo,ref);
title(['T1: ',num2str(rlevel), ' : ISE:', num2str(ISE_t1), ', ITSE:' ,num2str(ITSE_t1),', IAE:' ,num2str(IAE_t1), ', ITAE:' ,num2str(ITAE_t1)])