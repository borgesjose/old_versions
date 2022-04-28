function [melhores , mel] = selecao_natural(pop,K)
        
  %SELECIONAR OS MELHORES(Scores Mais Altos):
        
  pop = sortrows(pop, 2);
  melhores = [];
  mel = [];
  for i=size(pop,1):-1:(size(pop,1)-K+1)    
        melhores = [melhores,pop(i)] ;
        mel = [mel;pop(i,:)];
    end
       
end