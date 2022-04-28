function F1 =  Torneio(populacao)
    indexes = [];
    for i=1:10
        torn = populacao(randi(length(populacao)),:);
    end
     torn = sortrows(torn,2);
     F1 = torn{end,1};
end