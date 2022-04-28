function [gw,w,arm]= Ident1(n, d, eps1, tc1, y);

%********** Calculo do Valor de Pico **************
arm=0;
for t=1:n,
   if y(t)>=arm arm=y(t); end;
end;

%**************  Calcula Passagens pelo Zero *******************
    
kk=0;
for t=2:n, 
   if ((y(t-1)>0) & (y(t)<0))       
      f0=y(t-1);
      f1=y(t);
%     interpol(t,f0,f1,kk,x);
      kk=kk+1;
      m=f1-f0;
      b=-m*(t-1)+f0;
      x(kk)=-b/m;
   end;
   if ((y(t-1)<0) & (y(t)>0)) 
      f0=y(t-1);
      f1=y(t);
      kk=kk+1;
%     interpol(t,f0,f1,kk,x);
      m=f1-f0;
      b=-m*(t-1)+f0;
      x(kk)=-b/m;
   end;
end;

%    calcula_periodo

p=0;
for i=1:kk-2, p=p+x(i+2)-x(i); end;
    p=((p/(kk-2))-1.5)*tc1;
    w=2*pi/p;
    gw=-(pi*sqrt(arm^2-eps1^2))/(4*d);
%end ;

