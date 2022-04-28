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
% [Kc,Ti,Td] = Ident_com_rele_ZN(Tamostra,nptos,d,eps);

%% Definições do Controlador: 

Am_min = 2; 
Am_max = 5;
Theta_m_min = 45;
Theta_m_max = 72;
     
%Aplicando o CN-PID-FG:

 turnpoint = 500;%floor(rand*nptos);
for i=1:nptos,
    if (i<=turnpoint)  ref(i)=4; end;
    if (i>turnpoint)   ref(i) = 1; end;
end ;

for i=1:nptos,
    if (i<=nptos/4)  ref(i)=10; end;
    if (i>nptos/4)   ref(i) = 50; end;
    if (i>nptos/2 & i<=3*nptos/4)  ref(i)=25; end;
    if (i>3*nptos/4)   ref(i) = 60; end;
end ;
clear y;
y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;


erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;

L=2;%Provavelmente o valor de limite das memberships functions

%%
%randpmo criado aleatoriamente por mim
%gene =[0.1920,0.0227,-0.0480,0.1057,0.0430,2.0568,-0.5208,0.2094,0.4995,-0.0960,0.8679,0.1128];
%gene =[0.0217,0.2742,0.5700,0.4982,1.5688,0.1099,0.0714,0.4563,0.5000,0.3720,0.3186,0.4872];
%gene = [0.4389,0.1521,0.2080,0.4996,0.3999,0.7297,0.8111,0.2885,0.2978,0.7047,0.7371,0.3781];
%gene = thebest{1:1}(1:12);
gene =[0.3815,0.0712,-0.1897,0.5445,0.3570,0.0683,0.3031,0.1015,0.0800,0.8443,0.1032,0.3097];%05/05/19
Param_input = gene;
Param_output = [0.6871,0.5871,0.03495,0.02495,0.1,0.00];;%[0.9871,0.8871,0.0495,0.0295,0.05,0.00];
rlevel =0;
ruido = rlevel*rand(1,nptos);

    for i=5:nptos,

         y(i) =-a1*y(i-1)-a2*y(i-2)+b1*u(i-1)+b2*u(i-2)+ruido(i);
          
         erro(i)=ref(i)- y(i);%+ruido(i); %Erro

         rate(i)=(erro(i)-erro(i-1));%/Tc; %Rate of erro


         %Aqui na chamada da função é possivel escolher entre linear input(LI) e não linear input(NLI)
         %Aqui na chamada da função é possivel escolher entre linear input(LI) e não linear input(NLI)
     %Am(i) = Inferencia_T2(erro(i),rate(i),L,Param_input,Param_output,'LI');
     Am(i) = Inferencia_T2_S(erro(i),rate(i),L,Param_input,Param_output,'NLI');
     %Am(i) = inferencia_ortodoxa_S(erro(i),rate(i),L,Param_input,Param_output,'NLI');
     %[Am(i),yl(i),yr(i),l(i),r(i)]=EIASC(R(:,1)',R(:,2)',Y(:,1)',Y(:,2)'); 
     %Am(i) = 1;
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
           tempo(i)=i*Tamostra;
            
          %fprintf('amostra:  %d \t entrada:  %6.3f \t saida:  %4.0f\n',i,u(i),y(i));

     end ;
H=nptos;
I_nli_t2 = esforco_ponderado(erro,u,H,100)
     ISE_t2 = objfunc(erro,tempo,'ISE')
     ITSE_t2 = objfunc(erro,tempo,'ITSE')
     ITAE_t2 = objfunc(erro,tempo,'ITAE')
     IAE_t2= objfunc(erro,tempo,'IAE')
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
grid;
plot(tempo,y,'b-');
hold on;
%plot(tempo,u,'g-');
% %%
% title(['T2,NLI: ISE:', num2str(ISE_t2), ', ITSE:' ,num2str(ITSE_t2),', IAE:' ,num2str(IAE_t2), ', ITAE:' ,num2str(ITAE_t2)])
% %title('Resposta ao degrau com mudança referência')
% %legend('Type-1-CN-PID-FG','Type-2-CN-PID-FG')
% plot(tempo,ref,'r--');
% xlabel('tempo(s)');
% ylabel('y(t)');
% % text1= {'T1: ISE:', num2str(ISE_t1), ', ITSE:' ,num2str(ITSE_t1),', IAE:' ,num2str(IAE_t1), ', ITAE:' ,num2str(ITAE_t1)}
% % text2= {'T2,IM-1: ISE:', num2str(ISE_t2), ', ITSE:' ,num2str(ITSE_t2),', IAE:' ,num2str(IAE_t2), ', ITAE:' ,num2str(ITAE_t2)}
% % text([100],[0.5],text2)
