
%% Load raw data from the txt files
laserscan_data      = readmatrix('simul_LASER_LASER_SIM.txt');
laserscan_times     = readmatrix('simul_LASER_LASER_SIM_times.txt');
N_laserscans        = size(laserscan_data, 1);
disp(['Loaded laserscans data']);
odometry_data       = readmatrix('simul_ODO.txt');
odometry_times      = readmatrix('simul_ODO_times.txt');
N_odometry          = size(odometry_data, 1);
disp('Loaded odometry data');
disp(['Number of laserscan entries: ', num2str(N_laserscans)])
disp(['Number of odometry entries:  ', num2str(N_odometry)])

if(size(laserscan_data, 1) ~= size(laserscan_times, 1))
    disp('WARNING: lasescar data and times have different number of elements!');
end
if(size(odometry_data, 1) ~= size(laserscan_times, 1))
    disp('WARNING: odometry data and times have different number of elements!');
end
if(size(laserscan_data, 2) ~= lidar_N)
    disp('WARNING: number of data in the laserscans is different from the one reported in configuration');
end


%% Organise data in cell arrays
laserscans  = cell(1, N_laserscans);            % cell-array with all LIDAR scans and other data
odometries  = cell(1, N_laserscans);            % cell-array with all LIDAR scans and other data
half_fov    = lidar_fov / 2;
angles      = linspace(-half_fov, half_fov, lidar_N) * pi / 180;   % LIDAR angles
times       = zeros(1, N_laserscans);           % time of each scan

x           = 0;                                % absolute odometry coordinate value
y           = 0;                                % absolute odometry coordinate value
theta       = 0;                                % absolute odometry coordinate value

% To make easier analysis, we assume a constant sampling time
dt_laserscan = mean(laserscan_times(2:end) - laserscan_times(1:end-1));
dt_odometry  = mean(odometry_times(2:end)  - odometry_times(1:end-1) );
dt           = mean([dt_laserscan, dt_odometry]);

for i = 1:N_laserscans 


    laserscans{i}       = Laserscan(laserscan_data(i, :), angles);  % create laserscan object
    odometries{i}.t     = (i-1) * dt;                               % quantized time
    odometries{i}.x     = odometry_data(i, 1);                      % save abs. odometry
    odometries{i}.y     = odometry_data(i, 2);                      % save abs. odometry
    odometries{i}.theta = odometry_data(i, 3);                      % save abs. odometry
    times(i)            = (i-1) * dt;                               % quantized time

end

clearvars x y theta angles i dt_laserscan dt_odometry laserscan_data odometry_data lidar_fov
clearvars laserscan_times odometry_times half_fov lidar_N N_laserscans N_odometry