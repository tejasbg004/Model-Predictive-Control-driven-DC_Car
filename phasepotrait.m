function phasepotrait(f,x1,x2,fig_no)

[x, y]= meshgrid(x1,x2);

t=0;

x1dot=zeros(numel(x),1);
x2dot=zeros(numel(x),1);
for i=1:numel(x)
xdot=f(t,[x(i);y(i)]);

x1dot(i)=xdot(1,1);
x2dot(i)=xdot(2,1);

end

x1dot=reshape(x1dot,size(x));
x2dot=reshape(x2dot,size(y));

figure(fig_no)
hold on 
quiver(x,y,x1dot,x2dot,'r');
figure(gcf)
xlabel('x1dot')
ylabel('x2dot')

box;

axis tight equal;
end