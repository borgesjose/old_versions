function escolha = Roleta_Russa(P)

    SUM = cumsum(P);
    sorte = rand;
    
        for i=1:length(P)
            if sorte<=SUM(i)
                escolha = i;
                break;
            end
        end
   
end
