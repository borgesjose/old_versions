function J=objfunc(Y,T,idx)
%
% Computation of performance indices ISE, IAE, ITSE and ITAE.
% Inputs: Y (step response of the closed loop system)
%         T (time vector)
%
% Author: Guilherme Barreto
% Date: 07/09/2018

error=Y;  % compute error vector

if isequal(idx,'ISE'),
	J=sum(error.^2);
end

if isequal(idx,'IAE'),
	J=sum(abs(error));
end

if isequal(idx,'ITSE'),
	J=sum(T.*(error.^2));
end

if isequal(idx,'ITAE'),
	J=sum(T.*abs(error));
end

end

