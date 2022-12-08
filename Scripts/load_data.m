
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
    disp('WARNING: lasescar data and times have different number of elements!')
end
if(size(odometry_data, 1) ~= size(laserscan_times, 1))
    disp('WARNING: odometry data and times have different number of elements!')
end


%% Organise data in cell arrays
laserscans = cell(1, N_laserscans); % cell-array with all LIDAR scans and other data
angles     = -90:0.5:90;            % LIDAR angles

x           = 0;                    % absolute odometry coordinate value
y           = 0;                    % absolute odometry coordinate value
theta       = 0;                    % absolute odometry coordinate value

% To make easier analysis, we assume a constant sampling time
dt_laserscan = mean(laserscan_times(2:end) - laserscan_times(1:end-1));
dt_odometry  = mean(odometry_times(2:end)  - odometry_times(1:end-1) );
dt           = mean([dt_laserscan, dt_odometry]);

for i = 1:N_laserscans 
    
    laserscans{i}.r     = laserscan_data(i,:)';                     % copy polar measure
    laserscans{i}.xscan = cos(angles*pi/180) .* laserscan_data(i,:);% laserscan in polar coordinates
    laserscans{i}.yscan = sin(angles*pi/180) .* laserscan_data(i,:);% laserscan in polar coordinates
    laserscans{i}.t_odo = odometry_times(i);                        % set true laserscan time
    laserscans{i}.t_lid = laserscan_times(i);                       % set true odometry time
    laserscans{i}.t     = (i-1) * dt;                               % quantized time
    laserscans{i}.x     = x;                                        % save abs. odometry
    laserscans{i}.y     = y;                                        % save abs. odometry
    laserscans{i}.theta = theta;                                    % save abs. odometry
    
    x                   = x     + odometry_data(i,1);               % update odometry
    y                   = y     + odometry_data(i,2);               % update odometry
    theta               = theta + odometry_data(i,3);               % update odometry

end

clearvars x y theta angles i dt_laserscan dt_odometry