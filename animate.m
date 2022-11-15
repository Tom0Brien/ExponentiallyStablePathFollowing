function [] = animate(sys,res,play_video)
%% Unpack results
res.q = res.x(:,1:2);
res.p = res.x(:,3:4);
% Animate the 2DOF manipulator
res.t1 = res.q(:,1);
res.x1 = 0.5*sys.l1*cos(res.t1);
res.y1 = 0.5*sys.l1*sin(res.t1);
res.t2 = res.q(:,2);
res.x2 = sys.l1*cos(res.t1) + 0.5*sys.l2*cos(res.t2);
res.y2 = sys.l1*sin(res.t1) + 0.5*sys.l2*sin(res.t2);
res.x_ef = sys.l1*cos(res.t1) + sys.l2*cos(res.t2);
res.y_ef = sys.l1*sin(res.t1) + sys.l2*sin(res.t2);
if(play_video)
    fig8 = figure(8);
    hold on
    % axis equal
    xlim([-2 2]);
    ylim([-2 2]);
    for i=1:length(res.t)
        % clear figure from previous frame
        clf(fig8)
        hold on
        xlim([-2 2]);
        ylim([-2 2]);
        % plot  2DOF manipulator
        plot_beam(res.x1(i),res.y1(i),res.t1(i),sys)
        plot_beam(res.x2(i),res.y2(i),res.t2(i),sys)
        grid on;
        pause(0.0005)
    end
end
end

