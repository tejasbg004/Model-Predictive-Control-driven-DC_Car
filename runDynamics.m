clc;
clear;

dynamics=@RHS_DCcar;
dynamics_cons=@RHS_DCcar_cons;

%% Integrate the open loop trajectory in time

T=0.01;
final_time=12;
timespan=[0:T:final_time];

% initial state

Xinit=[0;0;0;0;0];
GR=2.5;% Gear ratio
V=12;% input voltage
Rw=3.2e-2; % wheel radius
[ts,xstate]=ode45(@(t,x)dynamics_cons(t,x,GR,Rw,V),timespan,Xinit);
Wback=xstate(:,2)/GR;
Wfront=xstate(:,4)/Rw;
figure(1)
hold on

plot(ts,Wback,'k',Linewidth=3)
plot(ts,Wfront,'b',Linewidth=3)
xlabel('time')
ylabel('Angular speed')
legend('back wheel angular velocity','front wheel angular velocity')

t_start = tic;

%% MPC and Optimization 

N=5;

h=0.1;
total_time_Steps=12/h; 

V0=10*ones(N,1); % Initial input guess for N-1 time steps

time_trajectory=0:h:(N-1)*h;




% desired speed for back wheel and front wheel should always be the same
% and avoid slip as much as possible

Wdesire= 0*ones((total_time_Steps),1);

alpha=50;
tme=0;
omega=0;
extra_time=20;
for i=1:(total_time_Steps)+extra_time
    if omega<=280
    omega=alpha*tme;
    Wdesire(i)=omega;
    else
        Wdesire(i)=280;
    
    end
    tme=tme+h;
end

Velocity_desire= Wdesire/GR*Rw;

wheel_speed_desire=[Wdesire Velocity_desire];



%   Weighting matrix Q1,Q2,....QN each of them will be diognal matrix at
%   each time instant therefore the big Q matrix will be a block diognal
%   matrix
state_dim=2; % dimension of the reduced state space
bigQ=eye(N*state_dim,N*state_dim);

time=0;
for i=1:N
    bigQ(state_dim*(i-1)+1,state_dim*(i-1)+1)=exp(i); %(1,1),(3,3),(5,5).... (more emphasis on controlling the velocity than controlling the position hence more weights on the second entry than the first
    bigQ(state_dim*(i-1)+2,state_dim*(i-1)+2)=0.001;  %(2,2),(4,4),(6,6)....
end

xstate_final=0*ones(total_time_Steps,5);
Voltage_applied=0*ones(total_time_Steps,5);

slip_constraint=@slipConstraint;


%time_simulation=0:h:(N-1)*h;

for i=1:total_time_Steps
time_simulation=(i-1)*h:h:(i+N-2)*h;

[solution, fval, exitflag, Output]= solveMPC(V0,Xinit,time_simulation,wheel_speed_desire(i:i+N-1,:),bigQ,GR,Rw,dynamics,slip_constraint);


% run the final code using calculated u at each time step
V=solution(1);
Voltage_applied(i)=V;

[ts,xstates]=ode45(@(t,x)dynamics_cons(t,x,GR,Rw,V),[time time+0.1],Xinit);
time=time+0.1;
Xinit=xstates(end,:);
xstate_final(i+1,:)=xstates(end,:);
end


Wback_final=xstate_final(:,2)/GR;
Wfront_final=xstate_final(:,4)/Rw;

figure(4)

plot([0:h:total_time_Steps*h],Wback_final,'k',Linewidth=3)
hold on
plot([0:h:total_time_Steps*h],Wfront_final,'b',Linewidth=3)
xlabel('time')
ylabel('Angular speed')
legend('back wheel angular velocity','front wheel angular velocity')


elapsed_time = toc(t_start);
fprintf('Elapsed time: %.4f seconds\n', elapsed_time);





    
