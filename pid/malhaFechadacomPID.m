%Disciplina: Controle Digital 2017
%Data: 09 de maio de 2017
%Professor: Otacílio da Mota Almeida

%Limpar o Workspace - INICIO
   clc;
   %Deletar todas as variáveis no Workspace
   clear;
   %Fechar todas as Janelas
   close all;
%Limpar o Workspace - FIM

%criar objeto porta serial - INICIO
  %Verificar em Gerenciador de Dispositivos
  s = serial('COM4','BaudRate',19200);
  %Ajustar o caractere final para '\n'
  s.terminator = 'LF';
%criar objeto porta serial - FIM

%Verificar as portas abertas pelo MATLAB - INICIO
   out = instrfind(s);
   if(out.status == 'closed')
      fclose(instrfind);
      %Abrir a comunicação Serial
      fopen(s);  
   end
 % Verificar as portas abertas pelo MATLAB - FIM
 Codigo = '1';
 % Codigo=0 para a Temperatura do resistor
 % Codigo=1 para Velocidade do Cooler

%Ajustar o período de amostragem
Ts = 0.1;

%Quantidade de amostras
Qde_amostras = 200;


%zerar PWM
%Canal pwm ventilador
fprintf(s,  sprintf( '2' ));
%Canal pwm resistor
fprintf(s,  sprintf( '3' ));

%delay 'Diferente' com etime() (mais preciso que pause())
tt = clock;
while etime(clock, tt) < 5
%não fazer nada enquanto o não passar de 5 segundos
end

%INJETA UM DEGRAU DE TENSÃO 

REF = 50; % Referência 80% do PWM

pwm(1) = REF;
figure(1);
%ganho = 8.1;
ganho = 1;
%malha fechada

Ku = 8.91;
Tu = 0.35;
Tamostra = 0.1;

saida(1) = 0;
saida(2) = 0;
saida(3) = 0;
u(1) = 0;



Ti = 0.5*Tu;
Kc = 0.6*Ku;
Td = Ti/4;

g0 = Kc*(1+(Td/Tamostra)+(Tamostra/Ti));
g1 = -Kc*(1+2*(Td/Tamostra));
g2 = Kc*Td/Tamostra;

for k=1:Qde_amostras
    
    tt = clock;
    %configurar pwm
    flushinput(s);
    flushoutput(s);
    
    fwrite(s,Codigo,'uint8');
    fwrite(s,pwm(k),'uint8'); %Envia valor de PWM para Motor
    fwrite(s,10,'uint8');  %Código quebra de linha é necessário
        
    %DELAY
    while etime(clock, tt) < Ts
    %não fazer nada enquanto o tempo de amostragem não terminar
    end
    
    resposta = fread(s,2); %
    
    saida(k) = double(bitor(bitshift(uint16(resposta(1)),8),uint16(resposta(2))))/(7*Ts); %conversão: int pra rps
    ref(k) = REF;   
    
    erro(k) = REF - saida(k);
    
    if k==1
        u(k)= Kc*(Tamostra/Ti)*ref(k) - g0*saida(k) - g1*saida(k-1) - g2*saida(k-2);
    else
        u(k)= u(k-1) + Kc*(Tamostra/Ti)*ref(k) - g0*saida(k) - g1*saida(k-1) - g2*saida(k-2);
    end
    
    pwm(k+1) = u(k);
    u(k+1) = u(k);
    
    %TESTE DE SATURAÇÃO
    if(pwm(k+1) > 100)
        pwm(k+1) = 100;
    else
        if(pwm(k+1) < 30)
           pwm(k+1) = 30; 
        end
    end
    
   % FIM DO TESTE DE SATURAÇÃO
   
   %DELAY
   while etime(clock, tt) < 5
   %não fazer nada enquanto o não passar de 5 segundos
   end
   
end

Tempo = [1:Qde_amostras]*Ts;
plot(Tempo,saida,'LineWidth',2);
hold on;

plot(Tempo,ref,'r','LineWidth',2);
drawnow
grid on; hold on;

if(Codigo == '1')
   %Conversão para Saida de Velocidade
   title('Gráfico da Velocidate');
   xlabel('Tempo (s)');
   ylabel('Velocidade (RPS)');
else
   %Conversão para Saida de temperatura
   title('Gráfico da Temperatura');
   xlabel('Tempo (s)');
   ylabel('Tensão (mV)'); 
end

savefig('Gráfico da Velocidate.fig');%salvar as figuras...

%zerar PWM
%Canal pwm ventilador
fprintf(s,  sprintf( '2' ));
%Canal pwm resistor
fprintf(s,  sprintf( '3' ));


fclose(s);


