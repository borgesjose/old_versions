function pop =  mutacao(populacao,prob_mutation)

    chromosome_lenght = size(populacao{1,1},2);
    populacao_size = size(populacao,1);
    score = 1./[populacao{:,2}];
    %Mutação:
     for k=1:floor(prob_mutation*size(populacao,1))
         %Selecionar cromossomo para mutação:
            P = score./sum(score);
            index =  Roleta_Russa(P);
            mutante = populacao{index,1};

         %Selecionar pontos de mutação:
            for j=1:8
                ponto = randi([1,chromosome_lenght]);
                u = -0.05 + (0.05+0.05)*rand;
                mutante(ponto) = mutante(ponto)+(u);
            end
       populacao{index,1}= mutante; 
     end
     pop = populacao;
end