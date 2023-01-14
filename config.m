% This files contains all the parameters used while running the SLAM algorithms


% 
%  __  __       _          ___     _                    _       _       _        
% |  \/  | __ _(_)_ __    ( _ )   | |    ___   __ _  __| |   __| | __ _| |_ __ _ 
% | |\/| |/ _` | | '_ \   / _ \/\ | |   / _ \ / _` |/ _` |  / _` |/ _` | __/ _` |
% | |  | | (_| | | | | | | (_>  < | |__| (_) | (_| | (_| | | (_| | (_| | || (_| |
% |_|  |_|\__,_|_|_| |_|  \___/\/ |_____\___/ \__,_|\__,_|  \__,_|\__,_|\__\__,_|
%                                                                                

load_precomputed_data = true; % Use precomputed features                  
plot_animation = false; % Plot the laserscans
GT = true; % Compare the grand truth

select_laserscans = 'simul_LASER_LASER_SIM_1.txt';
select_odometries = 'simul_ODO_1.txt';
select_laser_times = 'simul_LASER_LASER_SIM_times_1.txt';
select_odo_times = 'simul_ODO_times_1.txt';
select_GT = 'simul_GT_1.txt';
save_datas = true;

%  _                                                
% | |    __ _ ___  ___ _ __ ___  ___ __ _ _ __  ___ 
% | |   / _` / __|/ _ \ '__/ __|/ __/ _` | '_ \/ __|
% | |__| (_| \__ \  __/ |  \__ \ (_| (_| | | | \__ \
% |_____\__,_|___/\___|_|  |___/\___\__,_|_| |_|___/
%
% Main values associated to the laserscan data

lidar_fov       = 180;          % field of view of the lidar (in degrees)
lidar_N         = 361;          % number of data in each 

%  _____          _                    _____      _                  _   _             
% |  ___|__  __ _| |_ _   _ _ __ ___  | ____|_  _| |_ _ __ __ _  ___| |_(_) ___  _ __  
% | |_ / _ \/ _` | __| | | | '__/ _ \ |  _| \ \/ / __| '__/ _` |/ __| __| |/ _ \| '_ \ 
% |  _|  __/ (_| | |_| |_| | | |  __/ | |___ >  <| |_| | | (_| | (__| |_| | (_) | | | |
% |_|  \___|\__,_|\__|\__,_|_|  \___| |_____/_/\_\\__|_|  \__,_|\___|\__|_|\___/|_| |_|
%
% Parameters for the feature extraction algorithm; naming based on the source paper:
% https://journals.sagepub.com/doi/pdf/10.1177/1729881418755245

% Seeding -> creating segments
% Reduction -> joining segments
% Expansion -> esxpansion of segments
features_param = struct( ...
                'N_min',                8, ...    % Min number of points for a line        
                'N_back',               2, ...    % Number of points of overlapping
                'epsilon_seeding',      0.05, ... % threshold for the distance between the point and the line
                'delta_seeding',        0.05, ... % threshold for the distance between the point and the projection on the line
                'epsilon_reduction',    0.1, ...  % 
                'delta_reduction',      0.1, ...  %
                'epsilon_expansion',    0.06, ... %
                'delta_expansion',      0.1, ...  %
                'alpha_critic',         5, ...    % Admissible angle between segments 
                'L_min',                0.8, ...  % Minimum length for a segment while removing non proper segments
                'N_min_check',          10, ...   % Minimum number of points in a segment while removing non proper segments
                'hidden_tr',            0.1 ...   % Treshold on distance for an hidden wall
            );


%  __  __              
% |  \/  | __ _ _ __   
% | |\/| |/ _` | '_ \  
% | |  | | (_| | |_) | 
% |_|  |_|\__,_| .__/  
%              |_|     

map_param = struct( ...
              'buffer_length', 20, ...                            % length of the buffer for updating the map
              'malhanobis_for_correspondence', 1, ...             % distance for checking the correspondence
              'malhanobis_for_update_map', 0.4, ...               % distance for updating the map buffer
              'malhanobis_for_controlling_new_features', 0.2, ... % distance for controlling new features in update map
              'ratio_map_observation_closure', 2, ...             % Size of the map w.r.t. observations for searching closure
              'min_number_observations_closure', 8, ...           % Min number of observation for a loop closure
              'ratio_features_observation_closure', 0.8, ...      % Number of old features in the area w.r.t. number of obs.
              'max_centroid_distance_closure', 1.5, ...             % Max centroid admissible distance
              'max_testing_angle_closure', 20, ...                % Max testing rotation angle for the correspondences of loops
              'N_theta_closure', 5, ...                           % Number of testing angles
              'max_testing_displacement_closure', 0.4, ...        % Max testing displacement for the correspondences of loops
              'N_displacement_closure', 5, ...                    % Number of testing displacements
              'malhanobis_for_closure', 2, ...                    % distance for checking the closure
              'ratio_correspondence_closure', 0.8, ...            % Ratio of correspondence observation features in closuress
              'MAXX', 18, ...                                     % Maximum length of controlling area for closure
              'MINX', 7, ...                                      % Mminimum length of controlling area for closure
              'MAXY', 9, ...                                      % Maximum width of controlling area for closure
              'MINY', 5, ...                                       % Maximum width of controlling area for closure
              'displacement_after_closure', 10 ...                % Distance to be travelled to search for a second loop
            );

%   _____ _     __ 
%  | ____| | __/ _|
%  |  _| | |/ / |_ 
%  | |___|   <|  _|
%  |_____|_|\_\_|  
%   
min_distance_features = 0.8; % If 2 feattures are closer then they are collapsed into only one
plot_figure = false; % to plot the incremental map

%  ____       _           _   
% |  _ \ ___ | |__   ___ | |_ 
% | |_) / _ \| '_ \ / _ \| __|
% |  _ < (_) | |_) | (_) | |_ 
% |_| \_\___/|_.__/ \___/ \__|
%                             

%  _                    _                      _    
% | |    __ _ _ __   __| |_ __ ___   __ _ _ __| | __
% | |   / _` | '_ \ / _` | '_ ` _ \ / _` | '__| |/ /
% | |__| (_| | | | | (_| | | | | | | (_| | |  |   < 
% |_____\__,_|_| |_|\__,_|_| |_| |_|\__,_|_|  |_|\_\
%                                                   

%   ___      _                      _              
% / _ \  __| | ___  _ __ ___   ___| |_ _ __ _   _ 
% | | | |/ _` |/ _ \| '_ ` _ \ / _ \ __| '__| | | |
% | |_| | (_| | (_) | | | | | |  __/ |_| |  | |_| |
%  \___/ \__,_|\___/|_| |_| |_|\___|\__|_|   \__, |
%                                           |___/ 
%

               

