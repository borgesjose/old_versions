function f =invgauss(x,m)
%%
%% inverse Gaussian
%% Autor: jose borges 
%% Data: 27/02/2022
%%

f = exp(-(x-m).^2./(2*x.*m.^2));

end