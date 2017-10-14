function animate(t,z,p, speed)
    hold on
    axis([-2 2 0 10])
    axis equal % sets the X:Y aspect ratio 1:1; otherwise things will look stretched
    h_ground = plot([-1 1],[0 0],'k-','LineWidth',5);
    h_slider = plot([0 0], [0 10], 'k-', 'LineWidth', 3);
    h_wing    = plot([0],[0],'-o',...
                'LineWidth',3,...
                'MarkerEdgeColor','r',...
                'MarkerFaceColor','r',...
                'MarkerSize',6);

    tic                                             % start counter
    while toc < t(end)/speed                        % while there's real time left
        tsim = toc*speed;                           % determine the simulation time to draw
        zint = interp1(t',z',tsim', 'linear')';     % interpolate to get coordinates at that time
        draw_lines(zint,p,h_wing);
    end
    draw_lines(z(:,end),p,h_wing);
end

function draw_lines(z,p, h_wing)
    keypoints = keypoints_batwing(z,p);
    h_wing.XData = keypoints(1,:);
    h_wing.YData = keypoints(2,:);
    drawnow
end
