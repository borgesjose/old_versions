%Disciplina: Controle Digital 2017
%Data: 25 de setembro de 2017
%Professor: Otacílio da Mota Almeida
%Aluno: José Borges do Carmo Neto

%Limpar o Workspace - INICIO
   clc;
   %Deletar todas as variáveis no Workspace
   clear;
   %Fechar todas as Janelas
   close all;
%Limpar o Workspace - FIM

%Parte de simulação individual:
for i=1:10
    teste(i,:) = 0.5+2*rand(3,1);
end

%Consideramos a seguinte função de transferencia
%%
G = tf(1,[1,3,2,1]);
fig = 1;%variavel para controle das figuras...
%%
%Calculo de K
%K = 0.5+2*rand(3,1);
K = [2.326751712278039;1.764718492450819;0.695080809998819] %Valores distintos de K adotados
%% 

for n=1:3
    Gma(n) = K(n)*G;
    Gmf(n) = feedback(Gma(n),1);
end
s = tf('s');
%%
for n=1:3
    figure;
    step(Gma(n));
    title(['Figura ' num2str(fig) ' Resposta ao Degrau em malha aberta para K = ' num2str(K(n))] , 'FontWeight','bold');
    savefig(['Resposta ao Degrau em malha aberta para K = ' num2str(K(n)) '.fig'] );
    fig = fig+1;
end
%%
for n=1:3
    figure;
    step(Gmf(n),t);
    title(['Figura ' num2str(fig) ' Resposta ao degrau em malha fechada para K = ' num2str(K(n))] , 'FontWeight','bold');
    %savefig([' Resposta ao Degrau em malha fachada para K =' num2str(K(n)) '.fig'] );
    %fig = fig+1;
end
%%
for n=1:3 
    dados_m(n) = step(Gma(n));
end
%%
%malha fechada..
tempo_pico = [3.22,3.52,4.44];
valor_pico =[1.13,0.975,0.543];
tempo_subida = [1.73,1.93,2.54 ];
valor_subida = [0.59, 0.549,0.353];
tempo_assent = [26.8,22.2,14];
valor_assent = [0.712,0.65,0.417];

%malha aberta...
tempo_pico_ma = [5.86,6.07,6.14];
valor_pico_ma =[2.66,2.02,0.796];
tempo_subida_ma =[3.55,3.61,3.27];
valor_subida_ma = [1.97, 1.52,0.538];
tempo_assent_ma = [9.27,9.19,9.55];
valor_assent_ma = [2.37,1.8,0.701];

fig=1;
for n=1:3
    figure;
    step(Gma(n),'r');hold on;
    title(['Figura ' num2str(fig) ' Resposta ao Degrau em malha aberta para K = ' num2str(K(n))] , 'FontWeight','bold');
    plot( tempo_pico_ma(n), valor_pico_ma(n),'o');%overshoot
    plot(tempo_subida_ma(n),valor_subida_ma(n),'o');
    plot(tempo_assent_ma(n),valor_assent_ma(n),'o')
    legend('Resposta ao Degrau',['Sobresinal Máximo: ' num2str(valor_pico_ma(n))],['Tempo de Subida: ' num2str(tempo_subida_ma(n)) ' s' ],['Tempo de Acomodação: ' num2str(tempo_assent_ma(n)) ' s' ]);
    hold off;
    fig = fig+1;
    figure;
    step(Gmf(n));hold on;
    title(['Figura ' num2str(fig) ' Resposta ao Degrau em malha fechada para K = ' num2str(K(n))] , 'FontWeight','bold');
    plot( tempo_pico(n), valor_pico(n),'o');%overshoot
    plot(tempo_subida(n),valor_subida(n),'o');
    plot(tempo_assent(n),valor_assent(n),'o')
    legend('Resposta ao Degrau',['Sobresinal Máximo: ' num2str(valor_pico(n)) ' '],['Tempo de Subida: ' num2str(tempo_subida(n)) ' s'],['Tempo de Acomodação: ' num2str(tempo_assent(n)) ' s'] );
    hold off;
    fig = fig+1;
end

%% 1.3 Determinando a função de transferencia
%Analise dos graficos
    %isso aqui  é uma rotina pra analisar a respota ao degrau e me dar os
    %parametros necessarios pra estimar a FT como de primeira ordem
for n=1:3
    dt = 0.05;
    t = 0:dt:30;
    var = step(Gma(n),t);%dados dos graficos ate um tempo de 30s...
    dy = diff(var)/dt;
    [m,p] = max(dy);
    yi(n) = var(p);%amplitude de inflexão
    ti(n) = t(p);%tempo do ponto de inflexão
    Kpo(n)= K(n);
    L(n) = ti(n) - yi(n)/m; %Atraso
    tau(n) = (var(end)-yi(n))/m+ti(n)-L(n); %Constantre de tempo
    
    a(n) = L(n)/m; %a é o parametro do método temporal
    
    % depois, usar a e L pra calcular os parametros:
    Kc(n) = 1.2/a(n);
    Ti(n) = 2*L(n);
    Td(n) = 0.5*L(n);
    Tp(n) = 3.4*L(n);
    
    %controlador:
    Contr(n) = Kc(n)*(1 + 1/(s*Ti(n)) + s*Td(n));
    
%     tang = m*(t - L(n));%reta tangente....
%     %plotar a reta tangente
%     figure;
%     step(Gma(n)); hold on;
%     plot(ti(n),yi(n),'d--r');
%     plot(t,tang,'r'); hold off;
end

    
%% 
    %Aproximando para um sistema de primeira ordem temos:
        s = tf('s');
        for n=1:3
            Gpo(n) = Kpo(n)*(exp(-s*L(n)))/(s*tau(n)+1);
        end
        
%% Comparando as respostas no tempo e na frequencia do modelo com o processo "real":

  % Resposta ao degrau:  
    fig = 7;
    for n = 1:1:3
        figure;
        step(Gma(n),20); hold on;
        step(Gpo(n),20); hold off;
        title(['Figura ' num2str(fig) ' Comparação das Respostas ao Degrau para K = ' num2str(K(n))],'FontWeight','bold');ylabel('G(s)'); xlabel('s');
        legend('Real','Aproximado');
        %savefig(['Comparação das Respostas ao Degrau para K = ' num2str(K(n)) '.fig'] );
        fig=fig+1;
    end
    
%%    
 %Resposta no  dominio da frequencia:
 fig=10;
    for n = 1:1:3
        figure;
        grid
        nyquist(Gma(n)); hold on;
        nyquist(Gpo(n)); hold off;
        title(['Figura ' num2str(fig) ' Comparação dos Diagramas de nyquist para K= ' num2str(K(n))] , 'FontWeight','bold');ylabel('Im'); xlabel('Re');
        legend('Real','Aproximado');
        savefig(['Comparação dos Diagramas de nyquist para K= ' num2str(K(n)) '.fig'] );
        fig=fig+1;
    end
    
%% 1.4 Projetar um controlador PID por Ziegler-Nichols
%Analise dos graficos
    %isso aqui  é uma rotina pra analisar a respota ao degrau da minha
    %estimativa....
for n=1:3
    dt = 0.05;
    t = 0:dt:30;
    var = step(Gpo(n),t);%dados dos graficos ate um tempo de 30s...
    dy = diff(var)/dt;
    [mpo,ppo] = max(dy);
    yi(n) = var(ppo);%amplitude de inflexão
    ti(n) = t(ppo);%tempo do ponto de inflexão
    Kpo(n)= K(n);
    Lpo(n) = ti(n) - yi(n)/mpo; %Atraso
    taupo(n) = (var(end)-yi(n))/mpo+ti(n)-Lpo(n); %Constantre de tempo
    apo(n) = Lpo(n)/mpo;%valor de a aproximado para a inclinação do ponto de inflexão
    
    %linha(n) = line([yi(n)/mpo ti(n) ], [0 yi(n)]);
    
    % depois, usar a e L pra calcular os parametros:
    Kcpo(n) = 1.2/apo(n);
    Tipo(n) = 2*Lpo(n);
    Tdpo(n) = 0.5*Lpo(n);
    Tppo(n) = 3.4*Lpo(n);
    
    %aí vc monta o controlador para o modelo estimado do sistema:
    Contrpo(n) = Kcpo(n)*(1 + 1/(s*Tipo(n)) + s*Tdpo(n));
    
end

%% Aplicando o controlador
fig= 13;
for n=1:3
    RealContr(n) = series(Contr(n),Gma(n));
    EstContr(n) = series(Contrpo(n),Gpo(n));
    
    figure; step(RealContr(n),30); hold on;
    step(EstContr(n),30); hold off;
    
    title(['Figura ' num2str(fig) ': Controlador PID em MA, real versus primeira ordem - K= ' num2str(K(n))] , 'FontWeight','bold');ylabel('Im'); xlabel('Re');
    legend('Real','1ª ordem');
    
    formA(n+3) = stepinfo(RealContr(n));
    formF(n+3) = stepinfo(EstContr(n));
    
    fig = fig +1;
end
%%
fig = 16;
for n=1:3
    
    figure; step(feedback(RealContr(n),1),30); hold on;
    step(feedback(EstContr(n),1),30); hold off;
    
    title(['Figura ' num2str(fig) ': Controlador PID em MF, real versus primeira ordem - K= ' num2str(K(n))] , 'FontWeight','bold');ylabel('Im'); xlabel('Re');
    legend('Real','1ª ordem');
    
    formA(n+6) = stepinfo(RealContr(n));
    formF(n+6) = stepinfo(EstContr(n));
    fig = fig +1;
end

%% Resposta no  dominio da frequencia:
 fig=31;
    for n = 1:1:3
        figure;
        grid
        nyquist(Gmf(n)); hold on;
        nyquist(RealContrfr(n)); hold off;
        title(['Figura ' num2str(fig) ': Nyquist Função normal e controlada - K = ' num2str(K(n))] , 'FontWeight','bold');ylabel('Amplitude'); xlabel('Tempo');
        legend('Não controlada','Controlada');
        fig=fig+1;
    end

%% 1.5: Projeto de controlador PID com o uso de relé sem histerese:

% processo discreto:

for n=1:3
    ftz(n) = c2d(Gma(n),dt);
end

deg = 0.3;
d = 1;
ep = 0.0;%relé sem histerese
fig=19;
for n=1:3
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
            e(i) = deg - y(i);
        end
        
        %aplicar o relé com histerese:
        if( abs(e(i)) < ep  && u(i-1) == d )
            u(i) = d;
        end
        if( abs(e(i)) < ep && u(i-1) == -d)    
            u(i) = -d;
        end
        if( e(i) > ep )
            u(i) = d;
        end
        if( e(i) < -ep )
            u(i) = -d;
        end 
    end
end
% plotar os gráficos de saída, controle e erro, respectivamente:
        
%         tempo = (1:1:it)*dt;
%         figure;
%         plot(tempo,ref);hold on;
%         plot(tempo,y);hold on;
%         plot(tempo,u);hold on;
%         plot(tempo,e);hold off;
%         legend('referência','saida','controle','erro');
%         title(['Figura ' num2str(fig) ': Sinais de Controle, Saida , Referência e Erro - K= ' num2str(K(n))] , 'FontWeight','bold');ylabel('Im'); xlabel('Re');
%         %title(['graficos para K=' num2str(K(n))])
%         %saveas(gcf,['graficos para K=' num2str(K(n)) '.jpg' ]);
%         fig = fig+1
end

%%
    Tu(1) = 11.05 - 6.5;
    Tu(2) =  15.95 - 11.35;
    Tu(3) = 17.55 - 12.75;

    a(1) = (0.8549 - (-0.4227))/2;
    a(2) = (0.7169 - (-0.2716))/2;
    a(3) = (0.4161 - (-0.03953))/2;
  
    for m=1:3
        A = a(m);%amplitude da oscilação do sinal.
        %d = (1.3-(-1.3))/2;%amplitude do rele.
        ep = 0.0;%indica que o rele é sem histerse
        Ku(m) = (4*d)*((sqrt(A^2 - ep^2)/A) + (1i*ep/A))/(pi*A) % estimativa do ganho crítico.
        num = [-1];
        den = [Ku(m)];
        fc(m) = tf(num,den);
        MG(m) = -20*log10(1/Ku(m));
    end
%% --- Sintonia por Ziegler-Nichols
for m=1:3
Kcfma(m) = 0.6*Ku(m)
Tifma(m) = 0.5*Tu(m)
Tdfma(m) = Tifma(m)/4

contfreq(m) = Kcfma(m)*(1 + 1/(s*Tifma(m)) + s*Tdfma(m));
end

%% Aplicação do PID frequencial:
fig = 22;
for n=1:3
    RealContrfr(n) = feedback(series(contfreq(n),Gma(n)),1);
    
    figure; 
    step(Gmf(n),t); hold on;
    step(RealContrfr(n),t); hold on;
    
    title(['Figura ' num2str(fig) ': Malha Fechada normal e controlada - K = ' num2str(K(n))] , 'FontWeight','bold');ylabel('Amplitude'); xlabel('Tempo');
    legend('Não controlada','Controlada');

    
    %formA(n+3) = stepinfo(RealContrfr(n));
    %formF(n+3) = stepinfo(EstContr(n));
    
    fig = fig +1;
end

%% Resposta no  dominio da frequencia:
 fig=25;
    for n = 1:1:3
        figure;
        grid
        nyquist(Gma(n)); hold on;
        nyquist(series(contfreq(n),Gma(n))); hold off;
        ylim([-10 1]);
        xlim([-2 2]);
        title(['Figura ' num2str(fig) ': Nyquist Função normal e controlada - K = ' num2str(K(n))] , 'FontWeight','bold');ylabel('Amplitude'); xlabel('Tempo');
        legend('Não controlada','Controlada');
        fig=fig+1;
    end



%% Aplicação do relé com histerese
% processo discreto:

for n=1:3
    ftz(n) = c2d(Gma(n),dt);
end

deg = 0.3;
d = 1;
ep = 0.15;%relé com histerese
fig=28;
for n=1:3
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
            e(i) = deg - y(i);
        end
        
        %aplicar o relé com histerese:
        if( abs(e(i)) < ep  && u(i-1) == d )
            u(i) = d;
        end
        if( abs(e(i)) < ep && u(i-1) == -d)    
            u(i) = -d;
        end
        if( e(i) > ep )
            u(i) = d;
        end
        if( e(i) < -ep )
            u(i) = -d;
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
        title(['Figura ' num2str(fig) ': Sinais de Controle, Saida , Referência e Erro - K= ' num2str(K(n))] , 'FontWeight','bold');ylabel('Im'); xlabel('Re');
        %title(['graficos para K=' num2str(K(n))])
        %saveas(gcf,['graficos para K=' num2str(K(n)) '.jpg' ]);
        fig = fig+1
end

%%
    Tu(1) = 12.8 - 7.5;
    Tu(2) =  13.35 - 7.9 ;
    Tu(3) = 16.55 - 10.05;

    a(1) = (1.088 - (-0.6707))/2;
    a(2) = (0.9439 - (-0.5327))/2;
    a(3) = (0.5793 - (-0.2074))/2;
  
    for m=1:3
        A = a(m);%amplitude da oscilação do sinal.
        %d = (1.3-(-1.3))/2;%amplitude do rele.
        ep = 0.15;
        Ku(m) = (4*d)*((sqrt(A^2 - ep^2)/A) + (1i*ep/A))/(pi*A); % estimativa do ganho crítico.
        fc(m) = -1/Ku(m);
        rp(m)=abs(fc(m)); 
        fip(m)=angle(-fc(m));
        omega(m) = 2*pi/Tu(m);
    end
    
    
%% --- Sintonia por Astrom

%*********Especificações*************
fim=40;
rs=8*rp; % alterar para ajuste de overshoot.
fis=pi*fim/180;

%*************Cálculo dos Parâmetros do Controlador***********
for n=1:3
    Kc(n)=rs(n)*cos(fis-fip(n))/rp(n);     
    aux1=sin(fis-fip(n))/cos(fis-fip(n));
    aux2=sqrt(1+aux1^2);
    aux3=aux1+aux2;
    Td(n)=aux3/(2*omega(n));
    Ti(n)=4*Td(n);
end

for n=1:3
    contrast(n) = (1 + 1/(s*Ti(n)) + s*Td(n))*Kc(n);
end



%% Aplicação do PID frequencial:
fig = 28;
for n=1:3
    RealContrast(n) = feedback(series(contrast(n),Gma(n)),1);
    
    figure; 
    step(Gmf(n),t); hold on;
    step(RealContrast(n),t); hold on;
    
    title(['Figura ' num2str(fig) ': Malha Fechada normal e controlada - K = ' num2str(K(n))] , 'FontWeight','bold');ylabel('Amplitude'); xlabel('Tempo');
    legend('Não controlada','Controlada');

    
    %formA(n+3) = stepinfo(RealContrfr(n));
    %formF(n+3) = stepinfo(EstContr(n));
    
    fig = fig +1;
end
%% Resposta no  dominio da frequencia:
 fig=31;
    for n = 1:1:3
        figure;
        grid
        nyquist(Gma(n)); hold on;
        nyquist(series(contfreq(n),Gma(n))); hold off;
        ylim([-10 1]);
        xlim([-2 2]);
        title(['Figura ' num2str(fig) ': Nyquist Função normal e controlada - K = ' num2str(K(n))] , 'FontWeight','bold');ylabel('Amplitude'); xlabel('Tempo');
        legend('Não controlada','Controlada');
        fig=fig+1;
    end

    
%%


    