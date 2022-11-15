function sys = model()
% MODEL Generates a structure defining a 2DOF robotic manipulator
%% Symbolic variables
syms q1 q2 p1 p2
q_sym = [q1 q2].';
p_sym = [p1 p2].';

%% Parameters
sys.n = 2; % Number of states
% kinematic parameters
sys.l1 = 1;
sys.l2 = 1;
sys.G = @(q) [1 -1; 0 1];

% inertial parameters
sys.m1 = 3;
sys.m2 = 3;
sys.j1 = 3/12;
sys.j2 = 3/12;
sys.M = @(q) [sys.j1 + (sys.l1^2*sys.m1)/4 + sys.l1^2*sys.m2,   (sys.l1*sys.l2*sys.m2*cos(q(1) - q(2)))/2;
                (sys.l1*sys.l2*sys.m2*cos(q(1) - q(2)))/2,                      (sys.m2*sys.l2^2)/4 + sys.j2];
            
% friction parameters
sys.D = @(q) eye(2); 

% Potential parameters
sys.g = 9.8;
sys.V = @(q) sys.g*sys.m2*(sys.l1*sin(q(1)) + (sys.l2*sin(q(2)))/2) + (sys.g*sys.l1*sys.m1*sin(q(1)))/2;

% System energy
sys.H = @(q,p) 0.5*p.'*(sys.M(q)\p) + sys.V(q);
sys.T = @(q,p) 0.5*p.'*(sys.M(q)\p);

% energy gradients
sys.dVdqSYM = matlabFunction(jacobian(sys.V(q_sym),q_sym).','vars',q_sym);
sys.dVdq = @(q) sys.dVdqSYM(q(1),q(2));
sys.dHdqSYM =  matlabFunction(jacobian(sys.H(q_sym,p_sym),q_sym).','vars',[q_sym, p_sym]);
sys.dHdq = @(q,p) sys.dHdqSYM(q(1),q(2),p(1),p(2));
sys.dHdp = @(q,p) sys.M(q)\p;
end

