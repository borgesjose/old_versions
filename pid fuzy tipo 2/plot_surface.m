%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piauí                       %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -José Borges do Carmo Neto-          %
% @author José Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  -- Level control of a conical tank using Interval  %
%  Type-2 Fuzzy Logic PID controllers for the Phase   %
%  and Gain Margins of the System                     % 
%  nas margens de fase e de ganho                     %
%  -- Version: 1.0  - 20/02/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%

L = 2;

x1=-2*L:0.1:2*L;
x2=-2*L:0.1:2*L;

Am_min = 1; 
Am_max = 5;

A = zeros(length(x1),length(x2));
tipo = 'T2';

%%

for i=1:length(x2), % De 1 até o No. total de medidas da variavel linguistica...
 
   for j=1:length(x1),
     
     if tipo == 'T1'
      Am(j)=Inferencia(x1(j),x2(i),L,'LI');
     end
     
     if tipo =='T2'
      Am(j) = Inferencia_T2(x1(j),x2(i),L,Param,'NLI');  % pertinencia aos conjuntos fuzzy (curvatura)
      Am(j) = Ami*Am_max + Am_min*(1 - Ami);
     end
     
   end
 A(i,:) = Am;
end
%%
figure;
surf(x1,x2,A)
