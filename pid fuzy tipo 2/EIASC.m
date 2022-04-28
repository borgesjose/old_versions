function [y,yl,yr,l,r]=EIASC(Xl,Xr,Wl,Wr,needSort)

%
% function [y,yl,yr,l,r]=EIASC(Xl,Xr,Wl,Wr,needSort)
%
% function to implement the EIASC algorithm in:
%
% D. Wu and M. Nie, "Comparison and Practical Implementation of Type-Reduction Algorithms for Type-2 
% Fuzzy Sets and Systems," IEEE International Conference on Fuzzy Systems, Taipei, Taiwan, June 2011.
%
% Dongrui WU, GE Research (drwu09@gmail.com), 7/18/2010
%
% Xl: A row vector containing the lower bounds of x
% Xr: A row vector containing the upper bounds of x
% Wl: A row vector containing the lower bounds of w
% Wr: A row vector containing the upper bounds of w
% needSort: “1” if at least one of Xl and Xr is not in ascending order. 
%           “0” if both Xl and Xr are in ascending order. Default “1.”
%y: (yl+yr)/2
%yl: lower bound of the type-reduced output
%yr: upper bound of the type-reduced output
%l: switch point for yl
%r: switch point for yr

ly=length(Xl); XrEmpty=isempty(Xr);
if XrEmpty;  Xr=Xl; end
if max(Wl)==0
    yl=min(Xl); yr=max(Xr);
    y=(yl+yr)/2;  l=1; r=ly-1; return;
end
index=find(Wr<10^(-10));
if length(index)==ly
    yl=min(Xl); yr=max(Xr);
    y=(yl+yr)/2; l=1; r=ly-1; return;
end
Xl(index)=[]; Xr(index)=[];
Wl(index)=[]; Wr(index)=[];
if nargin==4;  needSort=1; end

% Compute yl
if  needSort
    [Xl,index]=sort(Xl); Xr=Xr(index);
    Wl=Wl(index); Wr=Wr(index);
end
Wl2=Wl; Wr2=Wr;
for i=length(Xl):-1:2 % Make Xl unique
    if Xl(i)==Xl(i-1)
        Wl(i)=Wl(i)+Wl(i-1);
        Wr(i)=Wr(i)+Wr(i-1); Xl(i)=[];
        Wl(i-1)=[]; Wr(i-1)=[];
    end
end
ly=length(Xl);
if ly==1
    yl=Xl;  l=1;
else
    yl=Xl(end); l=1;
    a=Xl*Wl'; b=sum(Wl); 
    while l < ly && yl > Xl(l)
        a=a+Xl(l)*(Wr(l)-Wl(l));
        b=b+Wr(l)-Wl(l); 
        yl=a/b;   l=l+1;
    end
end

% Compute yr
if ~XrEmpty && needSort==1
    [Xr,index]=sort(Xr);
    Wl=Wl2(index); Wr=Wr2(index);
end
if ~XrEmpty
    for i=length(Xr):-1:2 % Make Xr unique
        if Xr(i)==Xr(i-1)
            Wl(i)=Wl(i)+Wl(i-1);
            Wr(i)=Wr(i)+Wr(i-1); Xr(i)=[];
            Wl(i-1)=[]; Wr(i-1)=[];
        end
    end
end
ly=length(Xr);
if ly==1
    yr=Xr; r=1;
else
    r=ly; yr=Xr(1); 
    a=Xr*Wl'; b=sum(Wl);
    while r>0 && yr < Xr(r)
        a=a+Xr(r)*(Wr(r)-Wl(r));
        b=b+Wr(r)-Wl(r); 
        yr=a/b;  r=r-1;
    end
end
y=(yl+yr)/2;