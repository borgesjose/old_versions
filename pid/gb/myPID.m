function [Y T]=myPID(Kp,Ki,Kd,Gp,t)
%
% PID controller for plant model Gp
% Parameters: Kp,Ki,Kd (controller gains)
% Plant transfer function: Gp
%
% Autor: Guilherme Barreto
% Data: 07/09/2018

% Funcao de transferencia do controlador: Gc(s) = NUM(s)/DEN(s)
numc=[Kd Kp Ki]; 
denc=[1 0];  
Gc=tf(numc,denc);

Gcp=series(Gc,Gp); % Funcao de transferencia do ramo direto: Gc(s)*Gp(s)

Gcl=feedback(Gcp);  % Funcao de transferencia em malha fechada

[Y T]=step(Gcl,t); % Comando opcional
