%% Implementing the optimization algorithm
function [solution, fval, exitflag, output]  = solveMPC(V,xinit,time_simulation,xdesire,bigQ,GR,Rw,dynamics,slip_constraint)

% Defining upper and lower bounds of my function

lowbnd=0*ones(size(V));
upbnd=12*ones(size(V));

%C=[0,1,0,0,0; 0,0,0,1,0];
C=[0,1,0,0,0];

% define inequality and equality constraint and later add the constraints based on the
% problem


Ainq=[]; Binq=[]; Aeq=[]; Beq=[];
% options for solver
options = optimoptions(@fmincon, ...
    'Algorithm', 'active-set', ...
    'Display', 'iter', ...
    'MaxIterations', 1000, ...
    'OptimalityTolerance', 1e-6, ...
    'ConstraintTolerance', 1e-6, ...
    'StepTolerance', 1e-10, ...
    'UseParallel', true);
%@(V) slip_constraint(V,xinit,time_simulation)

[solution, fval, exitflag, output] = fmincon(@(V) costFunctionDefination(V), ...
    V, Ainq, Binq, Aeq, Beq, lowbnd, upbnd, [], options);

    function costFunction= costFunctionDefination(V)
    

    [ts,xstate]=ode45(@(t,x)dynamics(t,x, time_simulation,GR,Rw,V),time_simulation,xinit);

    %error state matrix
    reduced_states=C*xstate';
    error_state_matrix= reduced_states'-xdesire; % this (Nx2 matrix)

    % error vector
    error_state_matrixT=error_state_matrix'; % we need [x11,x12,x21,x22,x31,x32...] vector after vectorizing if we do not transpose we get [x11,x21,31... ] where x11 and x21 are position at time instance 1 and 2 respectievly, instead we would be needinmg position and velocity at time instance 1(note that is how we created bigQ matrix
    error_state_vector=error_state_matrixT(:);
    %slip= slip_constraint(V,xinit,time_simulation);



    costFunction=(error_state_vector')*bigQ*(error_state_vector);%+1e3*sum(slip.^2); % no h is being used here in the cost function for the reason that it might cause ill conditioning
    
end
end




