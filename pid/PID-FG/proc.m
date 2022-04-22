function [y,u]=proc(no,P1,P2,K,tc,d,eps1);

a0=K/(P1*P2);
a1=K/(-P1*(P2-P1));
a2=K/(-P2*(P1-P2));
x1=-exp(-P1*tc);
x2=-exp(-P2*tc);
x3=x1+x2;
x4=exp(-(P1+P2)*tc);

c0=a0+a1+a2;

c1=a0*x3+a1*(x2-1)+a2*(x1-1);

c2=a0*x4-a1*x2-a2*x1;
r0=1;
r1=x3;
r2=x4;


for t=1:no,
    y(t)=0; r(t)=1;
end;

E(1)=0; E(2)=0; 
y(1)=0 ; y(2)=0 ; y(3)=0 ; y(4)=0; y(5)=0;

u(1)=0 ; u(2)=0 ; u(3)=0 ; u(4)=0; u(5)=d;

for t=5:no,

   y(t)=-r1*y(t-1)-r2*y(t-2)+c0*u(t-2)+c1*u(t-3)+c2*u(t-4);
   E(t)=-y(t);

   if ((abs(E(t))>eps1) & (E(t)>0))  u(t+1)=d; end;
   if ((abs(E(t))>eps1) & (E(t)<0))  u(t+1)=-d; end;
   if ((abs(E(t))<eps1) & (u(t)==d))  u(t+1)=d; end;
   if ((abs(E(t))<eps1) & (u(t)==-d))  u(t+1)=-d; end;
   if (E(t)==eps1)  u(t+1)=d; end;
   if (E(t)==-eps1)  u(t+1)=-d; end;
  
end;

