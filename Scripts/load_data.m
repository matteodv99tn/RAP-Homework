
%% Load raw data from the txt files
laserscan_data      = readmatrix('simul_LASER_LASER_SIM.txt');
laserscan_times     = readmatrix('simul_LASER_LASER_SIM_times.txt');
disp(['Loaded laserscans data']);
odometry_data       = readmatrix('simul_ODO.txt');
odometry_times      = readmatrix('simul_ODO_times.txt');
disp('Loaded odometry data');
disp(['Number of data rows: ', num2str(size(laserscan_data, 1))])

if(size(laserscan_data, 1) ~= size(laserscan_times, 1))
    disp('WARNING: lasescar data and times have different number of elements!')
end
if(size(odometry_data, 1) ~= size(laserscan_times, 1))
    disp('WARNING: odometry data and times have different number of elements!')
end