function landing_trajec_plot(sol_L)
%% solution initialisation

r_L = sol_L.r;
u_L = sol_L.u;

%% Trajectory plot
% Plot
figure;
hold on;
grid on;

% Trajectory and thrust vector plot for Lander Module
plot3(r_L(1,:), r_L(2,:), r_L(3,:),'LineWidth', 2, 'Color', 'blue');
q = quiver3(r_L(1,:), r_L(2,:), r_L(3,:), u_L(1,:), u_L(2,:), u_L(3,:));
q.ShowArrowHead = 'off';
q.Color = 'blue';
q.LineWidth = 0.25;

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
%title(['3D Surface Trajectory Plot w/ Initial Velocity:[' num2str(v(:,1)') ']']);
ylim([-50,50])
view(0,0);
hold off;

%% 3D Animation

% figure
% grid on;
% xlabel('X Position (m)');
% ylabel('Y Position (m)');
% zlabel('Z Position (m)');
% title('Spacecraft Descent Trajectory Animation');
% hold on;
% 
% % Set axis limits for better visualization
% %axis([-500 500 -500 500 0 1500]);
% 
% view(3); % 3D view
% 
% % Create plot handles for trajectory, velocity, and thrust vectors
% trajectoryPlot = plot3(NaN, NaN, NaN, 'b-', 'LineWidth', 1.5);
% thrustQuiver = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 'r', 'AutoScale','off', LineWidth=2);
% 
% % Animate the trajectory
% for k = 1:N-1
%     % Update trajectory data
%     set(trajectoryPlot, 'XData', r(1, 1:k), 'YData', r(2, 1:k), 'ZData', r(3, 1:k));
% 
%     % % Update thrust vectors
%     % set(thrustQuiver, 'XData', r(1, 1:k), 'YData', r(2, 1:k), 'ZData', r(3, 1:k), ...
%     %     'UData', u(1, 1:k), 'VData', u(2, 1:k), 'WData', u(3, 1:k));
% 
%     % Pause to control the speed of the animation
%     if k < N
%         pause((time(k+1) - time(k)) / 20); % Assumes t is in seconds
%     end
% end
% 
% hold off;

end
