% L = 2;
% 
% x=-2*L:0.001:2*L;
% y=-2*L:0.001:2*L;
% 
% M=[];
% M1=[];
% 
% 
% i=5004;
%gene = [0.01,0.01,0.01,0.01,0.01,0.01,.01,.01,0.01,0.01,0.01,0.01,0.01,0.01,.01,.01];
%%
% Itype = 'LI';
% for i=1:length(x),
%     z(i) = Inferencia(x(i),y(i),L,Itype);
% end
% figure;
% plot(x,z)
% hold on;
% plot(x,Ami)
% title('Curva para IM-1')
% legend('Ami-tipo-1','Ami-tipo-2')
%%
clear; clc; close all;

L = 2;

x=-2*L:0.001:2*L;
y=-2*L:0.001:2*L;

M=[];
M1=[];
for i=1:length(x), % De 1 até o No. total de medidas da variavel linguistica...
  [mi, mo] = pertinencias(x(i),y(i),L,'LI');  % pertinencia aos conjuntos fuzzy (curvatura)
  M=[M; mi];
  M1=[M1; mo];
end
%%
figure; hold on
plot(x,M(:,1),'r-'); % gráfico conjunto fuzzy erro NEGATIVO
plot(x,M(:,2),'b-'); % gráfico conjunto fuzzy erro ZERO
plot(x,M(:,3),'m-'); % gráfico conjunto fuzzy erro POSITIVO
hold off
%axis([0 0.2 0 1.2]);
xlabel('ERRO');
legend('NEGATIVO','ZERO','POSITIVO')

figure; hold on
plot(y,M1(:,1),'r-'); % gráfico conjunto fuzzy rate NEGATIVO
plot(y,M1(:,2),'b-'); % gráfico conjunto fuzzy rate ZERO
plot(y,M1(:,3),'m-'); % gráfico conjunto fuzzy rate POSITIVO
hold off
%axis([0 0.2 0 1.2]);
xlabel('RATE');
legend('NEGATIVO','ZERO','POSITIVO')
%%
z=0:0.001:1;
M=[];
M1=[];

for i=1:length(z), % De 1 até o No. total de medidas da variavel linguistica...
  M=[M; logaritima_baixa(z(i))];
  M1=[M1; logaritima_alta(z(i))];
end

figure; hold on
plot(z,M1(:,1),'r-'); % gráfico conjunto fuzzy rate NEGATIVO
plot(z,M(:,1),'b-'); % gráfico conjunto fuzzy erro NEGATIVO
hold off