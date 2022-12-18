% This files contains all the parameters used while running the SLAM algorithms

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

FE_conf.Np      = lidar_N;      % number of points in each laserscan
FE_conf.eps     = 0.03;         % threshold from point to line
FE_conf.delta   = 0.1;          % threshold from point to point
FE_conf.Snum    = 6;            % number of laser points in seed-segment
FE_conf.Pmin    = 10;           % min. num. of laser points in a extracted line segment
FE_conf.Lmin    = 0.5;          % minimum segment line length









