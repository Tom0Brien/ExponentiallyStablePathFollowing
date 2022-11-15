function [] = plot_results(sys,ctrl,sim,res)
%% PLOT_RESULTS 
% Plots various figures for paper

%% Unpack results
res.q = res.x(:,1:2);
res.p = res.x(:,3:4);
res.H = zeros(length(res.t),1);
res.Hd = zeros(length(res.t),1);
res.qd = zeros(length(res.t),2);
res.qe = zeros(length(res.t),2);
for i=1:length(res.t)
    res.H(i) = sys.H(res.q(i,:).',res.p(i,:).');
    res.Hd(i) = ctrl.Hd(res.q(i,:).',res.p(i,:).');
    res.qp(i,:) = ctrl.qp(res.q(i,:)).';
    res.qe(i,:) = res.q(i,:) - res.qp(i,:);
end

%% Plot configuration vs time
fig1 = figure(1);
plot(res.t,res.q,res.t,res.qp,'--')
legend('q_1', 'q_2 desired','q_2', 'q_1 desired')
xlabel('time (s)')
ylabel('Configuration')
grid on
set(findall(fig1,'type','text'),'FontSize',11)
set(findall(fig1,'type','axes'),'FontSize',11)
set(findall(fig1,'type','line'),'linewidth',4)

%% Plot configuration
fig2 = figure(2);
plot(res.q(:,1),res.q(:,2),'LineWidth',4);
hold on;
plot(res.qp(:,1),res.qp(:,2),'--','LineWidth',4);
grid on;
legend('q','q_d')
xlabel('q_1')
ylabel('q_2')
axis([-1 10 -1.5 1.5])

%% Plot path with vector field
fig3 = figure(3);
[X,Y] = meshgrid(-4:0.1:4,-4:0.1:4);
Z = zeros(length(X),length(Y));
for i = 1:length(X)
    for j =1:length(Y)
    Z(i,j) = ctrl.Vd([X(i,j);Y(i,j)]);
    end
end
surf(X,Y,Z);
hold on;
x = linspace(-4,4,100);
y = zeros(size(x,2),1);
for i=1:size(x,2)
    y(i) = ctrl.qd(x(i));
end
plot3(x,y,zeros(length(x),1)+1, 'LineWidth',5,'Color','red');

%% Plot path with vector field
fig4 = figure(4);
grid on;
N = 4;
dt = 0.1;
[X,Y] = meshgrid(-1.5:dt:N,-1.5:dt:N);
U = ones(size(X));
V = ones(size(Y));
q1 = linspace(0,N,length(X));
qp_plot = ctrl.qp(q1);
axis([0 0.1 0 0.1])
q_d = zeros(length(X),2);
for i = 1:size(X)
q_d(i,:) = ctrl.qp(q1(i)).';
end
for i=1:size(X,1)
    for j=1:size(X,2)
        q_ij = [X(i,j);Y(i,j)];
        vdb_plot = ctrl.vdb(q_ij);
        U(i,j) = vdb_plot(1);
        V(i,j) = vdb_plot(2);
    end
end
plot(q_d(:,1),q_d(:,2),'LineWidth',4, 'Color','black');
hold on;
quiver(X,Y,U,V,'LineWidth', 1, "Color","blue", 'LineWidth',1)
axis([0 4 -1 1.5])
grid on;
xlabel("q_1" ,'FontSize', 25)
ylabel("q_2" ,'FontSize', 25)
legend('Desired path','Vector field','FontSize', 16)

%% Plot closed loop energy
fig5 = figure(5);
plot(res.t, log(res.Hd), 'LineWidth',2.5)
grid on
legend('log H_d')
xlabel('time (s)')
ylabel('Closed loop energy')
axis([0 1.5 -30 10])

%% Plot from various initial conditions
fig6 = figure(6);
% Specify set of initial conditions
pd = @(q) (sys.M(q)*ctrl.vdb(q)).';
X0 = [0.5, 1, pd([0.5;1]);
      0.5, -0.5, (sys.M([0.5;-0.5])*ctrl.vdb([0.5;-0.5])).';
      1, 1, (sys.M([1;1])*ctrl.vdb([1;1])).';
      1, 0,(sys.M([1;0])*ctrl.vdb([1;0])).';
      1.5, 0 ,(sys.M([1.5;0])*ctrl.vdb([1.5;0])).';
      1.5,1.5,(sys.M([1.5;1.5])*ctrl.vdb([1.5;1.5])).';
      2,0,(sys.M([2;0])*ctrl.vdb([2;0])).';
      2,1.5,(sys.M([2;1.5])*ctrl.vdb([2;1.5])).';
      ];
dt = 0.2;
[X,Y] = meshgrid(0:dt:5,-1:dt:1.5);
U = ones(size(X));
V = ones(size(Y));
for i=1:size(X,1)
    for j=1:size(X,2)
        q_ij = [X(i,j);Y(i,j)];
        vdb_plot = ctrl.vdb(q_ij);
        U(i,j) = vdb_plot(1);
        V(i,j) = vdb_plot(2);
    end
end
hold on;
quiver(X,Y,U,V,'LineWidth', 1, "Color","black", 'LineWidth',0.5)
for i=1:size(X0,1)
    % Run simulation
    ode = @(t,x) sim.dx(x(1:2),x(3:4),ctrl.u(t,x(1:2),x(3:4)));
    [t,x] = ode45(ode,[0 sim.t_end],X0(i,:).',odeset('RelTol',1e-9));
    % Plot output
    res.t = t;
    res.q = x(:,1:2);
    plot(res.q(:,1),res.q(:,2),'LineWidth',2.5);
    xlabel('q_1','FontSize',25)
    ylabel('q_2','FontSize',25)
end
x = 0:0.1:5;
plot(x,sin(x),'--','LineWidth',4,'Color','black');
axis([0 3 -1 1.5])
grid on;

%% Plot with different tuning parameters
fig7 = figure(7);
controllers = [
               controller(sys,1,1,0);
               controller(sys,1,1,1);
               controller(sys,2,1,1);
               controller(sys,2,2,1);
               controller(sys,2,2,2);
               ];
for i=1:size(controllers,1)
    % Run simulation
    ode = @(t,x) sim.dx(x(1:2),x(3:4),controllers(i).u(t,x(1:2),x(3:4)));
    [t,x] = ode45(ode,[0 10],[0;1;0;1],odeset('RelTol',1e-12));
    % Plot output
    res.t = t;
    res.q = x(:,1:2);
    res.p = x(:,3:4);
    res.Hd = zeros(length(res.t),1);
    for j=1:length(res.t)
        res.Hd(j) = controllers(i).Hd(res.q(j,:).',res.p(j,:).');
    end
    subplot(2,1,1);
    plot(res.q(:,1),res.q(:,2),'LineWidth',2.5);
    grid on;
    hold on;
    xlabel('q_1','FontSize',25);
    ylabel('q_2','FontSize',25);
    subplot(2,1,2);
    plot(res.t,log(res.Hd),'LineWidth',3);
    hold on;
    grid on;
    xlabel('t','FontSize',25);
    ylabel('log($$\tilde{H}_d$$)','FontSize',25,'Interpreter','latex');
end
subplot(2,1,1);
x = 0:0.1:10;
plot(x,sin(x),'--','LineWidth',2.5,'Color','black');
legend( 'K_p=1, K_d=1, K_{\alpha} = 0', ...
        'K_p=1, K_d=1, K_{\alpha} = 1', ...
        'K_p=2, K_d=1, K_{\alpha} = 1', ...
        'K_p=2, K_d=2, K_{\alpha} = 1', ...
        'K_p=2, K_d=2, K_{\alpha} = 2', ...
        'Desired path', ...
        'FontSize',16);
axis([0 4 -1 2])
subplot(2,1,2);
legend( 'K_p=1, K_d=1, K_{\alpha} = 0', ...
        'K_p=1, K_d=1, K_{\alpha} = 1', ...
        'K_p=2, K_d=1, K_{\alpha} = 1', ...
        'K_p=2, K_d=2, K_{\alpha} = 1', ...
        'K_p=2, K_d=2, K_{\alpha} = 2', ...
        'FontSize',16)
axis([0 4 -25 5])
set(gca,"FontSize",20)
end