
config

half_fov        = lidar_fov / 2;
angles          = linspace(-half_fov, half_fov, lidar_N) * pi / 180;


i = 5600;
laserscan_data  = readmatrix('simul_LASER_LASER_SIM.txt');
laserscans{i} = Laserscan(laserscan_data(i, :), angles);    % create laserscan object
laserscans{i}.extract_feature();                            % extract features


plot(laserscans{i})
title(['Time: ', num2str(laserscans_times(i)), 's']);