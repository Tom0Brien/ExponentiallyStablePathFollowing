function ctrl = controller(sys,Kp,Kd,K_alpha)
% CONTROLLER - Generates a controller with user provided gains Kp, Kd and K_alpha

% Symbolic variables
syms q1 q2 z1 z2 
q_sym = [q1 q2].';
z_sym = [z1 z2].';
n = sys.n ;

% Desired path
ctrl.qd = @(q) sin(q(1));
ctrl.qp = @(q) [q(1); ctrl.qd(q(1))];
ctrl.fz = @(q) [q(1); q(2) - [0,1]*ctrl.qp(q(1))];
ctrl.fzb = @(q) [zeros(1,1) eye(1)]*ctrl.fz(q);
ctrl.fz1 = @(q) [q(1)];
ctrl.fzi = @(z) [z(1); z(2) + [0,1]*ctrl.qp(z(1))];
ctrl.dfzidzSYM = matlabFunction(jacobian(ctrl.fzi(z_sym),z_sym),'vars',z_sym);
ctrl.dfzidz = @(z) ctrl.dfzidzSYM(z(1),z(2));

% Kinetic-potential energy function
ctrl.Kp = Kp*eye(n-1);
ctrl.Vd = @(q) 0.5*ctrl.fzb(q).'*ctrl.Kp*ctrl.fzb(q);
sys.dfzbdqSYM = matlabFunction(jacobian(ctrl.fzb(q_sym),q_sym),'vars',q_sym);
sys.dfzbdq = @(q) sys.dfzbdqSYM(q(1),q(2));
ctrl.dVddq = @(q) sys.dfzbdq(q).'*ctrl.Kp*ctrl.fzb(q);
ctrl.dVdDpt = @(q) zeros(2,1);

% Vector field
ctrl.phi = @(s) 1; % regulates speed on the path
ctrl.K_alpha = K_alpha*eye(n); 
ctrl.vpb = @(q) ctrl.dfzidz(ctrl.fz(q))*[1; 0]*ctrl.phi(ctrl.fz1(q));
ctrl.vdb = @(q) ctrl.vpb(q) - ctrl.K_alpha*ctrl.dVddq(q);

% Control law
ctrl.pd = @(q) sys.M(q)*ctrl.vdb(q);
ctrl.ptilde = @(q,p) p - ctrl.pd(q);
ctrl.dpddqSYM =  matlabFunction(jacobian(ctrl.pd(q_sym),q_sym),'vars',q_sym);
ctrl.dpddq = @(q) ctrl.dpddqSYM(q(1),q(2));

%closed loop energy
ctrl.Hd = @(q,p) 0.5*ctrl.ptilde(q,p).'*ctrl.ptilde(q,p) + ctrl.Vd(q);
ctrl.Kd = Kd*eye(n);
ctrl.ubar = @(t,q,p) - ctrl.Kd*ctrl.ptilde(q,p) - sys.M(q)\ctrl.dVddq(q);
ctrl.u = @(t,q,p) sys.G(q)\(sys.dHdq(q,p) + sys.D(q)*sys.dHdp(q,p) + ctrl.dpddq(q)*(sys.M(q)\p) + (-sys.D(q))*ctrl.ptilde(q,p) + ctrl.ubar(t,q,p));

end

