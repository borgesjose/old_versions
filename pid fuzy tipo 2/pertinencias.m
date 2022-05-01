function [mi1 ,mi2]=pertinencias(erro,rate,L,Itype)

% Retorna as pertinÃªncias da medida x aos conjuntos fuzzy
% definidos para as variaveis ERRO e RATE.
%
% Funcoes de pertinencia: - LINEARES: TRIANGULARES/TRAPEZOIDAS
%                         - NÃO-LINEARES: GAUSIANA E SIGMOID
%
% Data: 04/03/2019
% Autor: Jose Borges do Carmo Neto

%  L é o valor de av=bertura da MF zero;
%  T é um vetor com os valores dos parametros para IT2 

x=erro;
z=rate;

if isequal(Itype,'LI'),
    
    % Conjunto de ERROS NEGATIVOS
    if x<= -L,
       mi1(1)=1;
    elseif ((x> -L) & (x<0)),
       mi1(1)= -(1/L)*(x);
    else
       mi1(1)=0;
    end

    % Conjunto de ERROS ZEROS

    if ((x>-L) & (x<=0)),
       mi1(2)=(x/L)+1;
    elseif ((x>0) & (x<=L)),
       mi1(2)= -(x/L)+1;
    else
       mi1(2)=0;
    end

    % Conjunto de ERROS POSITIVOS
    if x>=L, % & x<=0.2,
       mi1(3)=1;
    elseif ((x>=0) & (x<=L)),
       mi1(3)=(x/L);
    else
       mi1(3)=0;
    end


    %%%%%%%%%
    %%%%%%%%%


    % Conjunto de RATE NEGATIVOS
    if z<= -L,
       mi2(1)=1;
    elseif ((z> -L) & (z<0)),
       mi2(1)= -(1/L)*(z);
    else
       mi2(1)=0;
    end

    % Conjunto de RATE ZEROS

    if ((z>-L) & (z<=0)),
       mi2(2)=(z/L)+1;
    elseif ((z>0) & (z<=L)),
       mi2(2)= -(z/L)+1;
    else
       mi2(2)=0;
    end

    % Conjunto de RATE POSITIVOS

    if z>=L, % & z<=0.2,
       mi2(3)=1;
    elseif ((z>=0) & (z<=L)),
       mi2(3)=(z/L);
    else
       mi2(3)=0;
    end

end

if isequal(Itype,'NLI'),

        % Conjunto de ERROS NEGATIVOS
    if x<= -L,
       mi1(1)=1;
    elseif ((x> -L) & (x<0)),
       mi1(1)= -(1/L)*(x);
    else
       mi1(1)=0;
    end

    % Conjunto de ERROS ZEROS

    if ((x>-L) & (x<=0)),
       mi1(2)=(x/L)+1;
    elseif ((x>0) & (x<=L)),
       mi1(2)= -(x/L)+1;
    else
       mi1(2)=0;
    end

    % Conjunto de ERROS POSITIVOS
    if x>=L, % & x<=0.2,
       mi1(3)=1;
    elseif ((x>=0) & (x<=L)),
       mi1(3)=(x/L);
    else
       mi1(3)=0;
    end


    %%%%%%%%%
    %%%%%%%%%


    % Conjunto de RATE NEGATIVOS
    if x<= -L,
       mi2(1)=1;
    elseif ((x> -L) & (x<0)),
       mi2(1)= -(1/L)*(x);
    else
       mi2(1)=0;
    end

    % Conjunto de RATE ZEROS

    if ((x>-L) & (x<=0)),
       mi2(2)=(x/L)+1;
    elseif ((x>0) & (x<=L)),
       mi2(2)= -(x/L)+1;
    else
       mi2(2)=0;
    end

    % Conjunto de RATE POSITIVOS

    if x>=L, % & x<=0.2,
       mi2(3)=1;
    elseif ((x>=0) & (x<=L)),
       mi2(3)=(x/L);
    else
       mi2(3)=0;
    end
    
    
end

end

