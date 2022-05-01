function Am=inferencia(erro,rate,L,Itype)
%%%
%%% Implementacao de um controlador Fuzzy baseado em margem de fase e de ganho 
%%% do tipo Gain Scheduling
%%%
%%% Autor: jose borges
%%% Data: 04/03/2019

[mi mo]=pertinencias(erro,rate,L,Itype);   % Pertinencias para variavel de entrada ERRO

% REGRA 1: 

R1 = mi(1)*mo(1);

% REGRA 2: 

R2 =mi(1)*mo(2)  ;

% REGRA 3: 

R3= mi(1)*mo(3);

% REGRA 4: 

R4 = mi(2)*mo(1);

% REGRA 5:

R5 = mi(2)*mo(2);

% REGRA 6: 

R6= mi(2)*mo(3);

% REGRA 7: 

R7 = mi(3)*mo(1);

% REGRA 8: 

R8 = mi(3)*mo(2);

% REGRA 9: 

R9= mi(3)*mo(3);

% Etapa de desfuzzificacao

Am = R1*(exp(-R1*4)) + R2*(exp(-R2*4)) + R3*(exp(-R3*4))+ R4*(1 - exp(-R4*4)) + R5*(1 - exp(-R5*4)) + R6*(1 - exp(-R6*4))+ R7*(exp(-R7*4)) + R8*(exp(-R8*4)) + R9*(exp(-R9*4));

end
