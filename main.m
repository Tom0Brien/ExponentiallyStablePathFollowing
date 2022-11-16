clear;clc;close all;
%% Build system model for 2DOF manipulator
sys = model();

%% Control law
ctrl = controller(sys,10,10,1);

%% System ODE
sim.dx = @(q,p,u) [zeros(2) eye(2); -eye(2) -sys.D(q)]*[sys.dHdq(q,p); sys.dHdp(q,p)] + [zeros(2); sys.G(q)]*u;
% Define simulation initial conditions and simulation length
sim.q0 = [1; 0];
sim.p0 = [0; 1];
sim.x0 = [sim.q0; sim.p0];
sim.t_end = 10;

%% Run simulation
sim.ode = @(t,x) sim.dx(x(1:2),x(3:4),ctrl.u(t,x(1:2),x(3:4)));
[res.t,res.x] = ode45(sim.ode,[0 sim.t_end],sim.x0,odeset('RelTol',1e-12));

%% Plot results and figures
plot_results(sys,ctrl,sim,res);

%% Animate results of simulation
animate(sys,res,false);