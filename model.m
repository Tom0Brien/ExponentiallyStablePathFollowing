function sys = model()
%% MODEL 
% Generates a structure defining a 2DOF robotic manipulator

%% Symbolic variables
syms q1 q2 p1 p2
sys.q_sym = [q1; q2];
sys.p_sym = [p1; p2];

%% System dimensions
sys.n = 2; 

%% Kinematic parameters
sys.l1 = 1;
sys.l2 = 1;
sys.G = @(q) [1 -1; 0 1];

%% Inertial parameters
sys.m1 = 3;
sys.m2 = 3;
sys.j1 = 3/12;
sys.j2 = 3/12;
sys.M = @(q) [sys.j1 + (sys.l1^2*sys.m1)/4 + sys.l1^2*sys.m2,   (sys.l1*sys.l2*sys.m2*cos(q(1) - q(2)))/2;
              (sys.l1*sys.l2*sys.m2*cos(q(1) - q(2)))/2,                      (sys.m2*sys.l2^2)/4 + sys.j2];
            
%% Friction parameters
sys.D = @(q) eye(2); 

%% Potential parameters
sys.g = 9.8;
sys.V = @(q) sys.g*sys.m2*(sys.l1*sin(q(1)) + (sys.l2*sin(q(2)))/2) + (sys.g*sys.l1*sys.m1*sin(q(1)))/2;

%% System energy
sys.H = @(q,p) 0.5*p.'*(sys.M(q)\p) + sys.V(q);
sys.T = @(q,p) 0.5*p.'*(sys.M(q)\p);

%% Energy gradients
sys.dVdq = matlabFunction(jacobian(sys.V(sys.q_sym),sys.q_sym).','vars',{sys.q_sym});
sys.dHdq = matlabFunction(jacobian(sys.H(sys.q_sym,sys.p_sym),sys.q_sym).','vars',{sys.q_sym, sys.p_sym});
sys.dHdp = @(q,p) sys.M(q)\p;
end

