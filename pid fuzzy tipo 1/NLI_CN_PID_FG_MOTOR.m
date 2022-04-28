% O processo: 
% Hz_planta =
%  
%     0.0092 z^-1 + 0.0580 z^-2
%   -----------------------------
%   1 - 0.7893 z^-1  - 0.1407 z^-2
Tc = 0.1;
z = tf('z', Tc )

b1 = 0.0092;
b2 = 0.0580;

a1 = -0.7893;
a2 = -0.1407;

ftz = (b1*(z^-1)+ b2*(z^-2))/(1 + a1*(z^-1)+ a2*(z^-2));

% figure;
% step(ftz,50)

%% Aplicando o relé:
Tc = 0.1;
Tamostra = Tc;
nptos = 1600;
Kc = 1.7519;
Ti = 0.6913;
Td = 0.1728;
nptos = 1600;
% d = 80;
% eps = 1;
% %[Kc,Ti,Td] = Ident_com_rele(Tamostra,nptos,d,eps);
%[Kc,Ti,Td] = Ident_com_rele_ZN(Tamostra,nptos,d,eps);
%% Definições de 

Am_min = 2; 
Am_max = 5;
Theta_m_min = 45;
Theta_m_max = 72;

%     %Margem de fase:
%%
%         Theta_m = (pi/2)*(1-(1/Am));
% 
%         [Kc; Ki; Kd] = (pi/(2*Am(i)*Lp))*[b;c;a];

%% Aplicando o CN-PID-FG:


 turnpoint = 500;%floor(rand*nptos);
for i=1:nptos,
    if (i<=turnpoint)  ref(i)=1; end;
    if (i>turnpoint)   ref(i) =2 ; end;
end ;

for i=1:nptos,
    if (i<=nptos/4)  ref(i)=10; end;
    if (i>nptos/4)   ref(i) = 50; end;
    if (i>nptos/2 & i<=3*nptos/4)  ref(i)=25; end;
    if (i>3*nptos/4)   ref(i) = 60; end;
end ;
y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;
cores = ['c-','m-','b-','g-','r-','k-','y-','c--','m--','b--']
%for w=1:10

erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;

L = 2;%Provavelmente o valor de limite das memberships functions

Param = [0.6871,0.0395,0];

for i=5:nptos,
%     if (i==550),r1 = - 1.909;r2 = 0.9109;  end
%     if (i==150),r1 = - 1.88;r2 = 0.88;  end
     
     y(i) =-a1*y(i-1)-a2*y(i-2)+b1*u(i-1)+b2*u(i-2)+ruido(i);
     
     erro(i)=ref(i)-y(i);%+ruido(i); %Erro
     
     rate(i)=(erro(i)-erro(i-1));%/Tc; %Rate of erro
     
     Am(i) = Inferencia(erro(i),rate(i),L,Param,'NLI');
     %Am(i) = 1;
     %Aqui na chamada da função é possivel escolher entre linear input(LI) e não linear input(NLI)            
      
      Ami = Am(i)*Am_max + Am_min*(1 - Am(i)); 
      

           
      %Controlador:

            alpha = (Kc/Ami)*(1+((Td/Ami)/Tamostra)+(Tamostra/(2*(Ti*Ami))));
            beta = -(Kc/Ami)*(1+2*((Td/Ami)/Tamostra)-(Tamostra/(2*(Ti*Ami))));
            gama = (Kc/Ami)*(Td/Ami)/Tamostra;
        
            u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2);
            if u(i)<0
                u(i) = 0;
            end
            if u(i)>100
                u(i) = 100;
            end
            %u(i)= u(i-1)+U_p(i)+U_i(i)+U_d(i);
       tempo(i)=i*Tamostra;
       
      %fprintf('amostra:  %d \t entrada:  %6.3f \t saida:  %4.0f\n',i,u(i),y(i));
      
 end ;
     H=nptos;
     I_nli_t1 = esforco_ponderado(erro,u,H,100)
     ISE_t1 = objfunc(erro,tempo,'ISE')
     ITSE_t1 = objfunc(erro,tempo,'ITSE')
     ITAE_t1 = objfunc(erro,tempo,'ITAE')
     IAE_t1 = objfunc(erro,tempo,'IAE')
%% figure;
%grid;
plot(tempo,y,'r-');
hold on;
%plot(tempo,u,'g-');
plot(tempo,ref);
% title(['T1: ISE:', num2str(ISE_t1), ', ITSE:' ,num2str(ITSE_t1),', IAE:' ,num2str(IAE_t1), ', ITAE:' ,num2str(ITAE_t1)])
figure;
plot(tempo,Am,'c-')
%end