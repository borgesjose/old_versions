%Inferencia com output singleton
function Am =Inferencia_T2_S(erro,rate,L,T,Param,Itype)
%%%
%%% Maquina de inferencia TYPE-2
%%% 
%%%
%%% Autor: jose borges 
%%% Data: 04/03/2019
big_u = Param(1);
big_l = Param(2);
medium_u = Param(3);
medium_l = Param(4);
small_u  = Param(5);
small_l  = Param(6);

[mi mo]=pertinencias_T2_NL(erro,rate,L,T,Itype);   % Pertinencias para variavel de entrada ERRO

% Forma do vetor mi
% mi = [Nb,Zb,Pb,Zu,Zl,Nu,Nl,Pu,Pl]
% Forma do vetor mo
% mo = [Nb,Zb,Pb,Zu,Zl,Nu,Nl,Pu,Pl]

%Base de Regras:

% REGRA 1: 

R(1,:) = [mi(7)*mo(7),mi(6)*mo(6)];

Y(1,:) = [small_l,small_u];
  
% REGRA 2: 

R(2,:) = [mi(7)*mo(5),mi(6)*mo(4)];

Y(2,:) = [medium_l,medium_u];

% REGRA 3: 

R(3,:)= [mi(7)*mo(9),mi(6)*mo(8)];

Y(3,:) = [small_l,small_u];
 
% REGRA 4: 

R(4,:) = [mi(5)*mo(7),mi(4)*mo(6)];

Y(4,:) = [big_l,big_u];
 
% REGRA 5:

R(5,:) = [mi(5)*mo(5),mi(4)*mo(4)];

Y(5,:) = [big_l,big_u];

% REGRA 6: 

R(6,:)= [mi(5)*mo(9),mi(4)*mo(8)];

Y(6,:) = [big_l,big_u];

% REGRA 7: 

R(7,:) = [mi(9)*mo(7),mi(8)*mo(6)];

Y(7,:) = [small_l,small_u];

% REGRA 8: 

R(8,:) = [mi(9)*mo(5),mi(8)*mo(4)];

Y(8,:) = [medium_l,medium_u];


% REGRA 9: 

R(9,:)= [mi(9)*mo(9),mi(8)*mo(8)];

Y(9,:) = [small_l,small_u];
 

    for i=1:9
       [y(i),yl(i),yr(i),l(i),r(i)]=EIASC(Y(i,1),Y(i,2),R(i,1),R(i,2),1);
       IF(i) = (R(i,1)*yl(i) + R(i,2)*yr(i))/2;   
    end
%[Am1,yl_,yr_,l_,r_]=EIASC(Y(:,1),Y(:,2),R(:,1),R(:,2),1);

Am = sum(IF);

%Am = R1*(exp(-R1*4)) + R2*(exp(-R2*4)) + R3*(exp(-R3*4))+ R4*(1 - exp(-R4*4)) + R5*(1 - exp(-R5*4)) + R6*(1 - exp(-R6*4))+ R7*(exp(-R7*4)) + R8*(exp(-R8*4)) + R9*(exp(-R9*4));

end