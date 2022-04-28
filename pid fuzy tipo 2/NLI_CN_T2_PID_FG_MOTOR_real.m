% O processo: 
% Hz_planta =
%  
%     0.0092 z^-1 + 0.0580 z^-2
%   -----------------------------
%   1 - 0.7893 z^-1  - 0.1407 z^-2
Tc = 0.1;

inicializa('COM4')
%clear;
Tc = 0.1;
Tamostra = Tc;

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
 d = 50;
 eps = 1;
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
    if (i<=nptos/4)  ref(i)=20; end;
    if (i>nptos/4)   ref(i) = 25; end;
    if (i>nptos/2 & i<=3*nptos/4)  ref(i)=30; end;
    if (i>3*nptos/4)   ref(i) = 15; end;
end ;
y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;



erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;

L = 2;%Provavelmente o valor de limite das memberships functions
%randpmo criado aleatoriamente por mim
gene =[0.1920,0.0227,-0.0480,0.1057,0.0430,2.0568,-0.5208,0.2094,0.4995,-0.0960,0.8679,0.1128];
%gene =[0.0217,0.2742,0.5700,0.4982,1.5688,0.1099,0.0714,0.4563,0.5000,0.3720,0.3186,0.4872];
gene = [0.4389,0.1521,0.2080,0.4996,0.3999,0.7297,0.8111,0.2885,0.2978,0.7047,0.7371,0.3781];
%gene = thebest{1:1}(1:12);
Param_input = gene;
Param_output = [0.7871,0.5871,0.0495,0.0295,0.05,0.00];

for i=5:nptos,
%     if (i==550),r1 = - 1.909;r2 = 0.9109;  end
%     if (i==150),r1 = - 1.88;r2 = 0.88;  end
     tt = clock;
                 if u(i-1)<0
                u(i-1) = 0;
            end
            if u(i-1)>100
                u(i-1) = 100;
            end
            set_pwm(u(i-1));
     
     y(i) = recebe_velocidade;
     
     erro(i)=ref(i)-y(i);%+ruido(i); %Erro
     
     rate(i)=(erro(i)-erro(i-1));%/Tc; %Rate of erro
     
     %Am(i) = Inferencia_T2(erro(i),rate(i),L,Param_input,Param_output,'LI');
     %Am(i) = Inferencia_T2_S(erro(i),rate(i),L,Param_input,Param_output,'LI');
     %Am(i) = inferencia_ortodoxa_S(erro(i),rate(i),L,Param_input,Param_output,'LI');
     
     %Aqui na chamada da função é possivel escolher entre linear input(LI) e não linear input(NLI)            
      
      %Ami = Am(i)*Am_max + Am_min*(1 - Am(i)); 
     Ami = 1; 
      %Controlador:

            alpha = (Kc/Ami)*(1+((Td/Ami)/Tamostra)+(Tamostra/(2*(Ti*Ami))));
            beta = -(Kc/Ami)*(1+2*((Td/Ami)/Tamostra)-(Tamostra/(2*(Ti*Ami))));
            gama = (Kc/Ami)*(Td/Ami)/Tamostra;
        
            u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2);
            

            
            %u(i)= u(i-1)+U_p(i)+U_i(i)+U_d(i);
       tempo(i)=i*Tamostra;
       
      %fprintf('amostra:  %d \t entrada:  %6.3f \t saida:  %4.0f\n',i,u(i),y(i));
      while etime(clock, tt) < Tamostra
       %não fazer nada enquanto o tempo de amostragem não terminar
       end
 end ;
 
     H=nptos;
     I_nli_t2 = esforco_ponderado(erro,u,H,1)
     ISE_t2 = objfunc(erro,tempo,'ISE')
     ITSE_t2 = objfunc(erro,tempo,'ITSE')
     ITAE_t2 = objfunc(erro,tempo,'ITAE')
     IAE_t2 = objfunc(erro,tempo,'IAE')
     
%% 
figure;
%grid;
plot(tempo,y,'r-');
hold on;
plot(tempo,u,'g-');
plot(tempo,ref);

% title(['T1: ISE:', num2str(ISE_t1), ', ITSE:' ,num2str(ITSE_t1),', IAE:' ,num2str(IAE_t1), ', ITAE:' ,num2str(ITAE_t1)])
figure;
plot(tempo,Am,'c-')

%%
set_pwm(0);
finaliza;

