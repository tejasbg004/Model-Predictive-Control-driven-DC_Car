function [c,ceq] = slipConstraint(V0,Xinit,time_simulation)

    GR = 2.5;
    rw = 3.2 * exp(-2);
    % Initial state: [i; omega; theta; v; x]


    % Simulate the system using ode45
    [~, X] = ode45(@(t, x) RHS_DCcar(t, x, time_simulation, GR, rw, V0), ...
                   time_simulation, Xinit);

    % Use final state to evaluate slip
    x2= X(:,2); % omega
    x4 = X(:,4); % velocity

    tol = 1e-10;
    s = (x2 ./ (GR * rw) - x4) ./ (abs(x4) + tol);

    % Nonlinear constraint: slip ratio should be zero
    c=[];
    ceq = s;      % weighted slip ≈ 0
end