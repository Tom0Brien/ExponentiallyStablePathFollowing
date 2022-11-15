function plot_beam(x,y,t,sys)
    plt.width = 0.1;
    plt.m1Cnr = [0 sys.l1 sys.l1 0 0;
                0 0 plt.width plt.width 0;] - [sys.l1/2; plt.width/2];
    % rotate to correct orientation
    plt.rot = [cos(t) -sin(t); sin(t) cos(t)];
    plt.trans = [x; y];
    plt.m1Cnr = plt.rot*plt.m1Cnr + plt.trans;
    plot(plt.m1Cnr(1,:),plt.m1Cnr(2,:),'Color','Black','LineWidth',2);
end