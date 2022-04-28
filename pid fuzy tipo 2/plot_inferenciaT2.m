L = 2;

x=-2*L:0.001:2*L;
y=-2*L:0.001:2*L;

for i=1:length(x), % De 1 até o No. total de medidas da variavel linguistica...
  Am(i) = Inferencia_T2(x(i),y(i),L,Param,'NLI');  % pertinencia aos conjuntos fuzzy (curvatura)
end

figure;
plot(x,Am,'k--')