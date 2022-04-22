% Implementacao de um sistema de controle PID discreto
% em malha fechada
%
% Planta discretizada via Espaco de Estados (SS - State Space)
%
% Parametros da planta: M=1kg, K=20N/m, F=1N, b=10N.s/m
%
% Autor: Guilherme Barreto
% Data: 23/10/2018

clear; clc; close all

Ts=0.001;  % Taxa de amostragem
Tsim=2;   % Tempo de simulacao
t=0:Ts:Tsim;  % Instantes de amostragem
h=Ts; % discretization step (Euler method)
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

% Na forma Espaco de Estados
%A=[0 1;-20 -10]; B=[0;1]; C=[1 0]; D=[0];
%[A1,B1,C1,D1]=tf2ss(nump,denp);
%sys=ss(A,B,C,D);
%[Y1 T1]=step(sys,t);

figure; plot(T1,Y1,'r-'); grid;
xlabel('tempo (segundos)');
ylabel('Amplitude degrau');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Discretizacao da planta %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Representacao SS discreto - Forma controlavel
A(1,1)=1; A(1,2)=h;
A(2,1)=-denp(3)*h; 
A(2,2)=1-denp(2)*h;
B=[0;h]; 
C=[1 0];

%%%%%%%%%%%%%%%%%%%%%%
%%% PID discreto %%%%%
%%%%%%%%%%%%%%%%%%%%%%
c1 = Kp + Ki*h + Kd/h;
c2 = -(Kp + 2*Kd/h);
c3 = Kd/h;

% Instantes 1 e 2 (condicoes iniciais)
x=[0;0];  % Estados iniciais
y=[0;0]; 
erro=[0;0];
u=[0;0];

for k=3:L,
  erro(k) = Yref(k) - y(k-1);   % Erro no instante k

	Du = c1*erro(k) + c2*erro(k-1) + c3*erro(k-2);  % Sinal de controle no instante k
  
  u(k) = u(k-1) + Du;
  
  x = A*x + B*u(k);  % Atualiza estados
  y(k)=C*x;
  
end

figure; plot(t,Yref,'r-',t,y,'b-'); grid;
xlabel('tempo (segundos)');
ylabel('Amplitude');
title('Resposta ao Degrau Unitario');



