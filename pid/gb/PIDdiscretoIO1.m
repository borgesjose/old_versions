% Implementacao de um sistema de controle PID discreto
% em malha fechada
%
% Planta discretizada via Representacao Entrada-Saida (IO, Input-Output)
%
% Parametros da planta: M=1kg, K=20N/m, F=1N, b=10N.s/m
%
% Autor: Guilherme Barreto
% Data: 23/10/2018

clear; clc; close all

Ts=0.01;  % Taxa de amostragem
Tsim=2;   % Tempo de simulacao
t=0:Ts:Tsim;  % Instantes de amostragem
h=0.01; % discretization step (Euler method)
L=length(t);
Yref=ones(1,L);  % Entrada degrau discreto

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Valores iniciais dos ganhos do controlador  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Kp=300;  % Ganho proporcional 
Ki=70;   % Ganho integral
Kd=0;    % Ganho derivativo 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plant transfer function %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nump=[1]; denp=[1 10 20];
Gp=tf(nump,denp); % Plant transfer function

%[Y1 T1]=step(Gp,t);  % Resposta malha aberta
[Y1 T1]=myPID(Kp,Ki,Kd,Gp,t);   % Resposta malha fechada

figure; plot(T1,Y1,'r-'); grid;
xlabel('tempo (segundos)');
ylabel('Amplitude degrau');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Discretizacao da planta %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Condicoes iniciais
aux=1+10*h+20*h*h;  % Termo usado na planta discreta via Euler
y=[0;0]; 
u=[0;0];
erro=[0;0];
somaErro=0;
for k=3:L,
  erro(k) = Yref(k) - y(k-1);   % Erro no instante k

  somaErro=somaErro+erro(k);  % Integral (acumulador) do erro 
  
	u(k) = Kp*erro(k) + Ki*h*somaErro + (Kd/h)*(erro(k)-erro(k-1));  % Sinal de controle no instante k
  
  % Planta discretizada pelo metodo de Euler
  y(k) = (2+10*h)*y(k-1) - y(k-2) + h*h*u(k);
  y(k) = y(k)/aux;
end

figure; plot(t,Yref,'r-',t,y,'b-'); grid;
xlabel('tempo (segundos)');
ylabel('Amplitude');
title('Resposta ao Degrau Unitario');



