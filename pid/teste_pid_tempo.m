%% 1.5: Projeto de controlador PID com o uso de relé sem histerese:

% processo discreto:

for n=1:3
    ftz(n) = c2d(Gma(n),dt);
end
%%
    Tu(1) = 11.05 - 6.5;
    Tu(2) =  11.35 - 15.95;
    Tu(3) = 12.75 - 17.55;

    a(1) = (0.8549 - (-0.4227))/2;
    a(2) = (0.7169 - (-0.2716))/2;
    a(3) = (0.4161 - (-0.03953))/2;
  
    for m=1:3
        A = a(m);%amplitude da oscilação do sinal.
        %d = (1.3-(-1.3))/2;%amplitude do rele.
        ep = 0.0;%indica que o rele é sem histerse
        Ku(m) = (4*d)*((sqrt(A^2 - ep^2)/A) + (1i*ep/A))/(pi*A) % estimativa do ganho crítico.
        % Nc =  4.7749;
        num = [-1];
        den = [Nc(m)];
        fc(m) = tf(num,den);
        MG(m) = -20*log10(1/Nc(m));
    end
%% --- Sintonia por Ziegler-Nichols
for m=1:3
Kc(m) = 0.6*Ku(m)
Ti(m) = 0.5*Tu(m)
Td(m) = Ti(m)/4

contfreq = Kcf(n)*(1 + 1/(s*Tif(n)) + s*Tdf(n));
end
%%
Tamostra = 0.05


deg = 0.3;
d = 1;
ep = 0.0;%relé sem histerese

u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;
%% Aplicação do PID frequencial:
for n=1:3
    
    g0=Kc*(1+(Td/Tamostra)+(Tamostra/Ti));
    g1=-Kc*(1+2*(Td/Tamostra));
    g2=Kc*Td/Tamostra;
    
    [num, den] = tfdata(ftz(n));
    num = cell2mat(num);
    den = cell2mat(den);

    it = 500;

for i=1:it
    ref(i) = deg;
    % valores iniciais: calculo manual
    if (i == 1)
        y(1) = 0;
        e(1) = 1;
        u(1) = d;
    else
        if (i == 2)
            y(2) =                   num(2)*u(1);
            e(2) = deg - y(2);
        end
        if (i == 3)
            y(3) =          num(3)*u(1) + num(2)*u(2) - den(2)*y(2) - den(3)*y(1);
            e(3) = deg - y(3);
        end
        
        %valores restantes
        if (i > 3)
            %eq das diferenças da função:
            y(i) = num(4)*u(i-3) + num(3)*u(i-2) + num(2)*u(i-1) - den(2)*y(i-1) - den(3)*y(i-2) - den(4)*y(i-3);
            e(i) = ref(i) - y(i);
            u(t)= u(t-1) + Kc*(Tamostra/Ti)*r(t) - g0*y(t) - g1*y(t-1) - g2*y(t-2);
        end
        
         
    end
end
% plotar os gráficos de saída, controle e erro, respectivamente:
        
        tempo = (1:1:it)*dt;
        figure;
        plot(tempo,ref);hold on;
        plot(tempo,y);hold on;
        plot(tempo,u);hold on;
        plot(tempo,e);hold off;
        legend('referência','saida','controle','erro');
        title(['graficos para K=' num2str(K(n))]);
        saveas(gcf,['graficos para K=' num2str(K(n)) '.jpg' ]);
end




