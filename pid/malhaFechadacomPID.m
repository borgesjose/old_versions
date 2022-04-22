%Disciplina: Controle Digital 2017
%Data: 09 de maio de 2017
%Professor: Otac�lio da Mota Almeida

%Limpar o Workspace - INICIO
   clc;
   %Deletar todas as vari�veis no Workspace
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
      %Abrir a comunica��o Serial
      fopen(s);  
   end
 % Verificar as portas abertas pelo MATLAB - FIM
 Codigo = '1';
 % Codigo=0 para a Temperatura do resistor
 % Codigo=1 para Velocidade do Cooler

%Ajustar o per�odo de amostragem
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
%n�o fazer nada enquanto o n�o passar de 5 segundos
end

%INJETA UM DEGRAU DE TENS�O 

REF = 50; % Refer�ncia 80% do PWM

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
    fwrite(s,10,'uint8');  %C�digo quebra de linha � necess�rio
        
    %DELAY
    while etime(clock, tt) < Ts
    %n�o fazer nada enquanto o tempo de amostragem n�o terminar
    end
    
    resposta = fread(s,2); %
    
    saida(k) = double(bitor(bitshift(uint16(resposta(1)),8),uint16(resposta(2))))/(7*Ts); %convers�o: int pra rps
    ref(k) = REF;   
    
    erro(k) = REF - saida(k);
    
    if k==1
        u(k)= Kc*(Tamostra/Ti)*ref(k) - g0*saida(k) - g1*saida(k-1) - g2*saida(k-2);
    else
        u(k)= u(k-1) + Kc*(Tamostra/Ti)*ref(k) - g0*saida(k) - g1*saida(k-1) - g2*saida(k-2);
    end
    
    pwm(k+1) = u(k);
    u(k+1) = u(k);
    
    %TESTE DE SATURA��O
    if(pwm(k+1) > 100)
        pwm(k+1) = 100;
    else
        if(pwm(k+1) < 30)
           pwm(k+1) = 30; 
        end
    end
    
   % FIM DO TESTE DE SATURA��O
   
   %DELAY
   while etime(clock, tt) < 5
   %n�o fazer nada enquanto o n�o passar de 5 segundos
   end
   
end

Tempo = [1:Qde_amostras]*Ts;
plot(Tempo,saida,'LineWidth',2);
hold on;

plot(Tempo,ref,'r','LineWidth',2);
drawnow
grid on; hold on;

if(Codigo == '1')
   %Convers�o para Saida de Velocidade
   title('Gr�fico da Velocidate');
   xlabel('Tempo (s)');
   ylabel('Velocidade (RPS)');
else
   %Convers�o para Saida de temperatura
   title('Gr�fico da Temperatura');
   xlabel('Tempo (s)');
   ylabel('Tens�o (mV)'); 
end

savefig('Gr�fico da Velocidate.fig');%salvar as figuras...

%zerar PWM
%Canal pwm ventilador
fprintf(s,  sprintf( '2' ));
%Canal pwm resistor
fprintf(s,  sprintf( '3' ));


fclose(s);


