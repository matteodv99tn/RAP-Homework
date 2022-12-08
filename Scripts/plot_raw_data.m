
%% Plot of the robot path estimated by the odometry data
% Extraction of the data
x_rob_plot      = zeros(1, N_laserscans);
y_rob_plot      = zeros(1, N_laserscans);
theta_rob_plot  = zeros(1, N_laserscans);
time            = zeros(1, N_laserscans);
for i = 1:N_laserscans 
    x_rob_plot(i)       = laserscans{i}.x;    
    y_rob_plot(i)       = laserscans{i}.y;
    theta_rob_plot(i)   = laserscans{i}.theta;
    time(i)             = laserscans{i}.t;
end

figure(1), clf, hold on;

subplot(2, 1, 1); % Robot path bidimensional plot
plot(x_rob_plot, y_rob_plot);
xlabel('x [m]');
ylabel('y [m]');
title('Robot path - odometry data');
set(gca,'DataAspectRatio',[1 1 1]);
ylim([-2,6]);
grid on;

subplot(2, 1, 2), hold on; % Robot odo. time evolution
plot(time, x_rob_plot,      'DisplayName', 'x');
plot(time, y_rob_plot,      'DisplayName', 'y');
plot(time, theta_rob_plot,  'DisplayName', 'theta');
legend();
title('Robot coordinates - time evolution');
xlabel('Time [s]');
ylabel('x,y [m] - theta [rad]');
grid on;

% clearvars x_rob_plot y_rob_plot theta_rob_plot time i


%% Histrograms for the odometry increments
figure(2), hold on, clf;

ax1 = subplot(3, 1, 1); % x increment plot
histogram(odometry_data(:,1));
title('odometry - x increment');

ax2 = subplot(3, 1, 2); % y increment plot
histogram(odometry_data(:,2));
title('odometry - y increment');

ax3 = subplot(3, 1, 3); % theta increment plot
histogram(odometry_data(:,3));
title('odometry - theta increment');
xlabel('Increment value');

linkaxes([ax1,ax2,ax3],'x');
clearvars ax1 ax2 ax3



