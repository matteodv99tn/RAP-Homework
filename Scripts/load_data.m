
if ~exist('ProcessedData', 'dir')
    mkdir('ProcessedData')
end


%  _                                                  _       _        
% | |    __ _ ___  ___ _ __ ___  ___ __ _ _ __     __| | __ _| |_ __ _ 
% | |   / _` / __|/ _ \ '__/ __|/ __/ _` | '_ \   / _` |/ _` | __/ _` |
% | |__| (_| \__ \  __/ |  \__ \ (_| (_| | | | | | (_| | (_| | || (_| |
% |_____\__,_|___/\___|_|  |___/\___\__,_|_| |_|  \__,_|\__,_|\__\__,_|
%                                                                      
if load_precomputed_data && exist('ProcessedData/laserscans.mat', 'file')

    fprintf('Loading data from ProcessedData/laserscans.mat... ');
    load('ProcessedData/laserscans.mat');
    fprintf('Done!\n');

else % must build the file
    
    fprintf('Generating laserscan data from Data/simul_LASER_LASER_SIM_2.txt\n');
    laserscan_data  = readmatrix('simul_LASER_LASER_SIM_2.txt');
    N_laserscans    = size(laserscan_data, 1);
    laserscans      = cell(1, N_laserscans);
    fprintf('Number of laserscans: %d\n', N_laserscans);

    half_fov        = lidar_fov / 2;
    angles          = linspace(-half_fov, half_fov, lidar_N) * pi / 180;

    N_percentage_steps = 10;
    percentage_status  = round(linspace(0, N_laserscans, N_percentage_steps+1));

    fprintf('Loading laserscans and pre-computing the features ...\n')
    j = 1;
    tic;
    for i = 1:N_laserscans

        if i > percentage_status(j)
            fprintf('%3.1f%%  ', 100 * j / N_percentage_steps);
            j = j + 1;
        end

        laserscans{i} = Laserscan(laserscan_data(i, :), angles);    % create laserscan object
        laserscans{i}.extract_feature();                            % extract features
    end
    elapsed_time = toc;
    fprintf('\nDone! Elapsed time: %4.2fs\n', elapsed_time);
    fprintf('Saving into ProcessedData/laserscans.mat... ');
    save('ProcessedData/laserscans.mat', 'laserscans', 'N_laserscans');
    fprintf('Done!\n');

end
clearvars laserscan_data half_fov angles N_percentage_steps percentage_status
clearvars i j elapsed_time lidar_fov lidar_N


%   ___      _                      _                    _       _        
%  / _ \  __| | ___  _ __ ___   ___| |_ _ __ _   _    __| | __ _| |_ __ _ 
% | | | |/ _` |/ _ \| '_ ` _ \ / _ \ __| '__| | | |  / _` |/ _` | __/ _` |
% | |_| | (_| | (_) | | | | | |  __/ |_| |  | |_| | | (_| | (_| | || (_| |
%  \___/ \__,_|\___/|_| |_| |_|\___|\__|_|   \__, |  \__,_|\__,_|\__\__,_|
%                                            |___/                        
if load_precomputed_data && exist('ProcessedData/odometries.mat', 'file')

    fprintf('Loading data from ProcessedData/odometries.mat... ');
    load('ProcessedData/odometries.mat');
    fprintf('Done!\n');

else % must build the file
    
    fprintf('Generating odometry data from Data/simul_ODO_2.txt\n');
    odometry_data   = readmatrix('simul_ODO_2.txt');
    N_odometries    = size(odometry_data, 1);
    odometries      = cell(1, N_odometries);
    fprintf('Number of odometry entries: %d\n', N_laserscans);

    for i = 1:N_odometries
        odometries{i} = Odometry(odometry_data(i, :));
    end
    fprintf('Saving into ProcessedData/odometries.mat... ');
    save('ProcessedData/odometries.mat', 'odometries', 'N_odometries');
    fprintf('Done!\n');

end
clearvars odometry_data i

%% Dimensionality check
if N_laserscans ~= N_odometries
    warning('Number of laserscans and odometry entries must be the same!');
end


%  _____ _                     
% |_   _(_)_ __ ___   ___  ___ 
%   | | | | '_ ` _ \ / _ \/ __|
%   | | | | | | | | |  __/\__ \
%   |_| |_|_| |_| |_|\___||___/
%          
if load_precomputed_data && exist('ProcessedData/times.mat', 'file')  
    
    fprintf('Loading time vectors from ProcessedData/times.mat... ');
    load('ProcessedData/times.mat');
    fprintf('Done!\n');

else % must build the file
    
    fprintf('Generating time vectors from Data/simul_LASER_LASER_SIM_times.txt ');
    fprintf('and Data/simul_ODO_times.txt\n');
    laserscans_times    = readmatrix('simul_LASER_LASER_SIM_times_2.txt');
    odometries_times    = readmatrix('simul_ODO_times_2.txt');
    dt_laserscans       = mean(laserscans_times(2:end) - laserscans_times(1:end-1));
    dt_odometries       = mean(odometries_times(2:end) - odometries_times(1:end-1));
    dt                  = mean([dt_laserscans, dt_odometries]);

    if abs(dt_laserscans - dt_odometries) < 1e-6
        laserscans_times = 0:dt:(N_laserscans-1)*dt;
        odometries_times = 0:dt:(N_odometries-1)*dt;
    else
        warning('Laserscans and odometries have different sampling times!')
        laserscans_times = 0:dt_laserscans:(N_laserscans-1)*dt;
        odometries_times = 0:dt_odometries:(N_odometries-1)*dt;
    end

    fprintf('Saving into ProcessedData/times.mat... ');
    save('ProcessedData/times.mat', 'laserscans_times', 'odometries_times', ...
                                    'dt', 'dt_laserscans', 'dt_odometries');
    fprintf('Done!\n');

end

clearvars load_precomputed_data