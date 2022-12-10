
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

clearvars x_rob_plot y_rob_plot theta_rob_plot time i


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


%% Plot of a laserscan
n_scan      = 1;
data        = zeros(2, 361);
data(1,:)   = laserscans{n_scan}.xscan(:);
data(2,:)   = laserscans{n_scan}.yscan(:);
R           = [0, -1; 1, 0];
data        = R * data;

figure(3), clf, hold on;
plot(data(1,:), data(2,:), '.');
plot([-1, 0, 1 -1], [-1, 2, -1, -1], 'r');
plot(0, 0, 'or');
title(['Laserscan ' num2str(n_scan) ' - time ', num2str(laserscans{n_scan}.t)]);

set(gca,'DataAspectRatio',[1 1 1]);
grid on;

clearvars n_scan data R

if plot_animation
    
    fs_plot = 10;
    fs_meas = 1 / dt;
    i_step  = round(fs_meas / fs_plot);
    figure(4), clf, hold on;
    for i = 1:i_step:length(laserscans)

        plot_pointcloud(laserscans{i});
        pause(dt);

    end

    clearvars fs_plot fs_meaas i_step i
end
clearvars plot_animation



%% Custom function implementation

function plot_pointcloud(laserscan)

    clf, hold on;
    
    data        = zeros(2, 361);
    data(1,:)   = laserscan.xscan(:);
    data(2,:)   = laserscan.yscan(:);
    R           = [0, -1; 1, 0];
    data        = R * data;

    plot(data(1,:), data(2,:), '.');
    plot(0.5*[-1, 0, 1 -1], 0.5*[-1, 2, -1, -1], 'r');
    plot(0, 0, 'or');
    title(['Laserscan - Time ', num2str(laserscan.t)]);
    xlim([-12, 12]);
    ylim([-2, 16]);

    set(gca,'DataAspectRatio',[1 1 1]);
    grid on;

end