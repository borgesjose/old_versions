function [mi1 ,mi2]=pertinencias(erro,rate,L,T,Itype)

% Retorna as pertinÃªncias da medida x aos conjuntos fuzzy
% definidos para as variaveis ERRO e RATE.
%
% Funcoes de pertinencia TRIANGULARES/TRAPEZOIDAS
%
% Data: 04/03/2019
% Autor: Jose Borges do Carmo Neto

%  L é o valor de abertura da MF zero;
%  T é um vetor com os valores dos parametros para IT2 

x= erro;
z= rate;

theta = T(1:6);
gama = T(6:12);

b = L+1;
c = -(L+1);
v=5;
m = 0;

g = L/4;
gu=(L/4) + theta(3);
gl=(L/4) - theta(4);

%INPUT SET LINEAR

% Conjunto de ERROS NEGATIVOS T2:

%Conjunto tipo 1 de base:  
   mi1(1)= 1-1./(1+exp(-v*x+c));

% Superior:
   mi1(6)= 1-1./(1+exp(-v*x+c+theta(1)));

% Inferior:
   mi1(7)= 1-1./(1+exp(-v*x+c-theta(2)));



% Conjunto de ERROS ZEROS T2

%Conjunto tipo 1 de base:
   mi1(2)= exp(-(1/2)*((x-m)/g).^2);


% Superior:
   mi1(4)= exp(-(1/2)*((x-m)/gu).^2);


% Inferior:
   mi1(5)= exp(-(1/2)*((x-m)/gl).^2);



% Conjunto de ERROS POSITIVOS

%Conjunto tipo 1 de base:   
   mi1(3)= 1./(1+exp(-v*x+b));

% Superior:
   mi1(8)= 1./(1+exp(-v*x+b+theta(5))); 


% Inferior:
   mi1(9)= 1./(1+exp(-v*x+b-theta(6)));

%%%%%%%%%
%%%%%%%%%
% Conjunto de RATE NEGATIVOS T2:

gu=(L/4) + gama(3);
gl=(L/4) - gama(4);

%Conjunto tipo 1 de base:

   mi2(1)= 1-1./(1+exp(-v*x+c));


% Superior:

   mi2(6)= 1-1./(1+exp(-v*x+c+gama(1))); 


% Inferior:

   mi2(7)= 1-1./(1+exp(-v*x+c-gama(2)));




% Conjunto de RATE ZEROS T2


%Conjunto tipo 1 de base:

   mi2(2)=exp(-(1/2)*((x-m)/g).^2);


% Superior:

   mi2(4)=exp(-(1/2)*((x-m)/gu).^2);


% Inferior:

   mi2(5)=exp(-(1/2)*((x-m)/gl).^2);



% Conjunto de RATE POSITIVOS

%Conjunto tipo 1 de base:

   mi2(3)= 1./(1+exp(-v*x+b));


% Superior:

   mi2(8)= 1./(1+exp(-v*x+b+gama(5)));  


% Inferior:

   mi2(9)= 1./(1+exp(-v*x+b-gama(6)));

   
end

