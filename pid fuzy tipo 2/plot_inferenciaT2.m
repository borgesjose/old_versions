L = 2;

x=-2*L:0.001:2*L;
y=-2*L:0.001:2*L;

Am_min = 1; 
Am_max = 5;

for i=1:length(x), % De 1 até o No. total de medidas da variavel linguistica...
  %Am(i) = Inferencia_T2(x(i),y(i),L,Param,'NLI');  % pertinencia aos conjuntos fuzzy (curvatura)
  Ami = Inferencia_T2_SB(x(i),y(i),L,Param,'NLI');
  Am(i) = Ami*Am_max + Am_min*(1 - Ami);
end

figure;
plot(x,Am,'k--')