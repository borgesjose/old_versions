function I = esforco_ponderado(e,u,H,sigma),
        
        erro_quadratico = e(1:H)*e(1:H)';
        
        for k=2:length(u),
        delta_u(k) = u(k)- u(k-1);
        end
        delta_u_quadratico = delta_u(1:H)*delta_u(1:H)';
        
        I = erro_quadratico + sigma*delta_u_quadratico;
        
end