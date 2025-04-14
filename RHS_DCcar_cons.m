% Dynamics for Car and DC motor

function ddt= RHS_DCcar_cons(t,x,GR,rw,V)

tol=1e-1;
%GR=2.5;
%rw=3.2*exp(-2);
a1=1.63;
c1=-27;
a2=-0.9;
c2=-15;
a3=-0.1;
m=1.14;
g=9.81;
Q=0.37;
R=0.141;
L=0.003;
Kb=0.00574;
Km=Kb;
b=3.97e-6;
J=1e-4;

s= (x(2)/GR*rw-x(4))/(abs(x(4))+tol);

if s>1
    s=1;
elseif s<-1
    s=-1;
end

if s<0
    s=abs(s);
    mu=-(a1*(1-exp(c1*s))+a2*(1-exp(c2*s))+a3*s);
else
    mu=(a1*(1-exp(c1*s))+a2*(1-exp(c2*s))+a3*s);
end




FT= mu*m*g*Q;

didt=-R/L*x(1)-Kb/L*x(2)+1/L*V;

dWdt=Km/J*x(1)-b/J*x(2)-rw*FT/(GR*J);

dthetadt=x(2);

dvdt=FT/m;

dxpdt=x(4);

ddt=[didt;dWdt;dthetadt;dvdt;dxpdt];
end