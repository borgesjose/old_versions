IIIIIIIIIIIIIIIIIIK^<p)^`````````````````````````````````````````````````%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
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


%clear; clc; close all;

x=-2*L:0.001:2*L;
y=-2*L:0.001:2*L;

L = 2;
M=[];
M1=[];

%gene = thebest{1:1}(1:16);
% 
 Param = [gene,1,1];
% 
 Itype = 'LI';
% for i=1:length(x)
%     Ami(i) = Inferencia_T2(x(i),y(i),L,Param,Itype);
% end
%%
for i=1:length(x), % De 1 até o No. total de medidas da variavel linguistica...
  %[mi, mo] = pertinencias_T2(x(i),y(i),L,Param,Itype);  % pertinencia aos conjuntos fuzzy (curvatura)
  [mi, mo] = pertinencias_T2_NL(x(i),y(i),L,Param,Itype);  % pertinencia aos conjuntos fuzzy (curvatura)
  M=[M; mi];
  M1=[M1; mo];
end

% Forma do vetor M
% M = [Nb,Zb,Pb,Zu,Zl,Nu,Nl,Pu,Pl]
% Forma do vetor M1
% M1 = [Nb,Zb,Pb,Zu,Zl,Nu,Nl,Pu,Pl]

%Plotagem


%PLOTAGEM DAS MF DO ERRO:
figure; hold on

plot(x,M(:,4),'-.b'); % gráfico conjunto fuzzy erro ZERO UPPER
plot(x,M(:,5),'-.b'); % gráfico conjunto fuzzy erro ZERO LOWER
x2 = [x, fliplr(x)];
inBetween = [M(:,5)', fliplr(M(:,4)')];
fill(x2, inBetween, 'g');
plot(x,M(:,2),'b-'); % gráfico conjunto fuzzy erro ZERO
legend('ZERO')

plot(x,M(:,6),'g-'); % gráfico conjunto fuzzy erro NEGATIVO UPPER
plot(x,M(:,7),'g-'); % gráfico conjunto fuzzy erro NEGATIVO LOWER
x2 = [x, fliplr(x)];
inBetween = [M(:,7)', fliplr(M(:,6)')];
fill(x2, inBetween, 'b');
plot(x,M(:,1),'r-'); % gráfico conjunto fuzzy erro NEGATIVO
legend('NEGATIVO')

plot(x,M(:,8),'g-'); % gráfico conjunto fuzzy erro POSITIVO UPPER
plot(x,M(:,9),'g-'); % gráfico conjunto fuzzy erro POSITIVO LOWER
x2 = [x, fliplr(x)];
inBetween = [M(:,9)', fliplr(M(:,8)')];
fill(x2, inBetween, 'y');
plot(x,M(:,3),'m-'); % gráfico conjunto fuzzy erro POSITIVO
legend('POSITIVO')

hold off
%axis([0 0.2 0 1.2]);
xlabel('ERRO');
legend('NEGATIVO','ZERO','POSITIVO')



%PLOTAGEM1 DAS MF DO RATE:
figure; hold on

plot(x,M1(:,4),'-.b'); % gráfico conjunto fuzzy RATE ZERO UPPER
plot(x,M1(:,5),'-.b'); % gráfico conjunto fuzzy RATE ZERO LOWER
x2 = [x, fliplr(x)];
inBetween = [M1(:,5)', fliplr(M1(:,4)')];
fill(x2, inBetween, 'g');
plot(x,M1(:,2),'b-'); % gráfico conjunto fuzzy RATE ZERO


plot(x,M1(:,6),'g-'); % gráfico conjunto fuzzy RATE NEGATIVO UPPER
plot(x,M1(:,7),'g-'); % gráfico conjunto fuzzy RATE NEGATIVO LOWER
x2 = [x, fliplr(x)];
inBetween = [M1(:,7)', fliplr(M1(:,6)')];
fill(x2, inBetween, 'b');
plot(x,M1(:,1),'r-'); % gráfico conjunto fuzzy RATE NEGATIVO


plot(x,M1(:,8),'g-'); % gráfico conjunto fuzzy RATE POSITIVO UPPER
plot(x,M1(:,9),'g-'); % gráfico conjunto fuzzy RATE POSITIVO LOWER
x2 = [x, fliplr(x)];
inBetween = [M1(:,9)', fliplr(M1(:,8)')];
fill(x2, inBetween, 'y');
plot(x,M1(:,3),'m-'); % gráfico conjunto fuzzy RATE POSITIVO

hold off
%axis([0 0.2 0 1.2]);
xlabel('RATE');
legend('NEGATIVO','ZERO','POSITIVO')
