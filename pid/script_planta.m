function [matriz_resultado, fis] = script_planta(fuzz, params)

format shortg
addpath(genpath('src'))
global t y r e de u du up pwm k SerPIC

n = params.n;
T = params.T;
Gz = params.Gz;
if ~isfield(params, 'K')
    K = dcgain(Gz);
else
    K = params.K;
end
if ~isfield(params, 'D')
    D = 0;
else
    D = params.D;
end
if ~isfield(params, 'limits')
    limits = struct();
else
    limits = params.limits;
end
sistema = params.sistema;
o = 3;              %início de amostragem
t = (0:(n-1))*T;    %vetor de tempo

%% I/O

[termina, leitura, escrita, planta] = inicializacom('COM5', sistema, Gz);
SerPIC.Timeout = 1;

%% CONFIGURAÇÃO

%ESTADO INCIAL
[r, y, e, de, u, du, up, pwm] = deal(zeros(n, 1)); 
ping = nan(n, 1);
t0 = tic;

for i = 1:length(params.mf)
    fuzzmf = params.mf{i};
    for j = 1:length(fuzzmf)
        [~, mf_name, x_limits] = fuzzmf{j}{:};
        limits.(mf_name) = struct('min', min(x_limits), 'max', max(x_limits));
    end
end
if ~isfield(limits, 'saida_permanente')
   limits.saida_permanente = struct('min', 0, 'max', 100); 
end
if ~isfield(limits, 'referencia')
   limits.referencia = struct('min', limits.saida_permanente.min*K, 'max', limits.saida_permanente.max*K);
end
if ~isfield(limits, 'saida_sistema')
   limits.saida_sistema = limits.referencia;
end

try
    if ~isfis(fuzz)
        fis = fis_vetor(fuzz, params);
    else
        fis = fuzz;
    end
catch
    if ~isstruct(fuzz)
        fis = fis_vetor(fuzz, params);
    else
        fis = fuzz;
    end
end

%% LOOP DE CONTROLE
for k = 3:n
    %LEITURA
    time = tic;
    y(k) = max(min(leitura(), limits.saida_sistema.max), limits.saida_sistema.min);

    %REFERÊNCIA E ERRO
    r(k) = max(min(params.referencia(k), limits.referencia.max), limits.referencia.min);
    e(k) = r(k) - y(k);    
    
    %SATURAÇÃO ERRO
    e(k) = max(min(e(k), limits.erro.max), limits.erro.min);
    
    %SATURAÇÃO RATE
    de(k) = -(y(k) - y(k-1))/T;
    de(k) = max(min(de(k), limits.rate.max), limits.rate.min);
    
    %CONTROLE
    du(k) = evalfis([e(k) de(k)], fis(1));
    if length(fis) == 2
        up(k) = evalfis(r(k), fis(2));
    else
        up(k) = r(k)/K;
    end
    
    %u(k) = du(k) + up(k);
    u(k) = u(k-1) + du(k);
    
    %SATURAÇÃO SAIDA
    pwm(k) = max(min(u(k), limits.saida_permanente.max), limits.saida_permanente.min);
    
    %ESCRITA
    escrita(pwm(k) + D);
    ping(k) = toc(time);
    
    %DELAY
    if ~isempty(planta)
        while toc(time) < T
        end
    end
end
termina();
matriz_resultado = [(r(o:n) - y(o:n))  t(o:n)'];

if params.plotar
    fprintf('Tempo: %f segundos\n\n', toc(t0) - toc(time));
    salvar = {'fis', 'limits', 't', 'y', 'r', 'e', 'de', 'u', 'du', 'up', 'pwm', 'ping', 'T'};
    pasta = ~isempty(planta)*'pratica' + isempty(planta)*'teorica';
    salvar_em = ['particula/' pasta '/' sistema];
    fig = plotudo(t(o:k), y, r, e, u, pwm, 0, 0, 0);
    hold on
    if ~exist(salvar_em, 'dir')
        mkdir(salvar_em);
    end
    date = datestr(datetime('now'));
    date(date == '-' | date == ':') = '_';
    path = [salvar_em '/' date];
    save([path '.mat'], salvar{:})
    saveas(fig, [path '.fig'])
    disp(['Plant: ' salvar_em ' Saved at: ' path])
end

end
     
