classdef Map < handle 

%     _   _   _        _ _           _            
%    / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
%   / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
%  / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
% /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
%                                                 
properties 

    landmark_vector;        % a vector of Landmark objects that represent the map
    landmark_buffer;        % a buffer of all Landmark objects seen by the robot in the "n" past 
                            % scans; this is used to perform the map update to add (or not) new 
                            % landmarks
    buffer_length;          % length of the landmark buffer
    buffer_i;               % index of the current position in the landmark buffer (used to 
                            % perform the circular buffer)
    grid_configuration;     % configuration parameters for the grid for the map update
                 
end % properties

%  ____        _     _ _        __  __                _                                                             
% |  _ \ _   _| |__ | (_) ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___                                           
% | |_) | | | | '_ \| | |/ __| | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|                                          
% |  __/| |_| | |_) | | | (__  | |  | |  __/ | | | | | |_) |  __/ |  \__ \                                          
% |_|    \__,_|_.__/|_|_|\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/                                          
%
% Even if Matlab do not provide an "easy" way to discriminate public and private member functions,
% here we firstly define the functions that are intended to be called in the main program                                                                                                                   
methods 

    function obj = Map() % constructor
        
        obj.buffer_length   = 10;
        obj.landmark_buffer = cell(1, obj.buffer_length);
        obj.buffer_i        = 1;
        obj.grid_configuration = struct( ...
                'LB_x',         -10, ...    % lower bound for the x coordinate of the map
                'UB_x',         10, ...     % upper bound for the x coordinate of the map
                'LB_y',         -10, ...    % lower bound for the y coordinate of the map
                'UB_y',         10, ...     % upper bound for the y coordinate of the map
                'dx',           0.1, ...    % grid step size w.r.t. x
                'dy',           0.1, ...    % grid step size w.r.t. y
                'threshold',    0.6,...     % threshold for the occupancy grid
                'max_sigma',    0.2 ...     % maximum value for the standard deviation of the 
                                    ...     % landmarks to be considered valid
            );

    end

    
    % Given the map and a robot placed inside it that provides a vector of observations, this
    % function should be able to generate:
    %  - the innovation vector z;
    %  - the Jacobian of the observation function H_x;
    %  - the covariance matrix R of the measurement.
    % Reference to equation taken from:
    % https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf
    function [z, H_x, R] = compute_innovation(map, robot, observation_vector)

        P1, P2 = obj.compute_correspondences(robot, observation_vector)

        if(length(P1) ~ length(P2))
            error([ 'The number of correspondences is not the same for P1 and P2,',
                    ' could not compute the innovation vector']);
        end

        % Initialization:
        dim_z   = 2 * length(P1);       % dimension of the observation vector
        dim_P   = 3 + 2 * map.size();   % dimension of the P matrix (robot pose + (2x)landmarks)

        z       = zeros(dim_z, 1);
        H_x     = zeros(dim_z, dim_P);
        R       = zeros(dim_z, dim_z);

        for k = 1:length(P1)

            i_obs       = P1(k);    % index of the measurement inside the observation vector
            i_land      = P2(k);    % index of the landmark inside the map
            observation = observation_vector(i_obs);    % current observation of interest
            landmark    = map.landmark_vector(i_land);  % current landmark of interest

            % Compute the estimated observation (with jacobian) bases on the current robot pose 
            % estimation and the landmark position estimation
            h, Jh_x_rob, Jh_x_land = robot.landmark_observation(landmark);

            % Fill the innovation vector, the Jacobian and the covariance matrix
            z(2*k-1:2*k)                            = observation.z - h;        % eq (19, 26)
            H_x(2*k-1:2*k, 1:3)                     = Jh_x_rob;                 % eq (25)
            H_x(2*k-1:2*k, 3+2*i_land-1:3+2*i_land) = Jh_x_land;                % eq (25)
            R(2*k-1:2*k, 2*k-1:2*k)                 = observation.R;

        end
    end


    % The objective of this function is to compute the correspondence between the landmarks 
    % (contained in the map object itself) and the observations coming from a robot.
    % Calling "O" the Nx1 vector of observation and "L" the Mx1 vector of landmarks, then the 
    % resulting vectors P1 and P2 should be two Jx1 vectors (of the same size). Note that we expect
    % in general J < N < M (the number of explored landmarks is higher then the number of 
    % observations, while not all observation can be associated to a landmark as there may be some
    % outliers).
    % With this premise, P1 and P2 are a sequence of pairs indexes relating an observation with a 
    % landmark, that is for the example
    %       P1 = [1; 3; 4]      and     P2 = [2; 4; 1]
    %   - the first observation is associated to the second landmark;
    %   - the third observation is associated to the fourth landmark;
    %   - the fourth observation is associated to the first landmark.
    function [P1, P2] = compute_correspondences(map, robot, observation_vector)
        
        %% TODO

    end


    % The goal of this function is to add the new observation to a buffer and check if it's possible
    % to add a new landmark to the map. 
    function new_landmarks = update_map(map, robot, observation_vector)

        map.add_to_buffer(robot, observation_vector);
        grid = map.initialize_grid();

        for i = 1:map.buffer_length
            grid = update_grid(grid, i, map.landmark_buffer{i}, map.grid_configuration);
        end

        candidates      = map.find_candidates(grid, map.grid_configuration);
        new_landmarks   = map.check_candidates(candidates);

        map.landmark_vector = [map.landmark_vector; new_landmarks];

    end

    function loop_closure(map, observation_vector)
        %% TODO
        
    end


    % Given a map it returns the number of landmarks, i.e. the length of the landmark vector
    function dim = size(map)
        dim = length(map.landmark_vector);
    end

%  ____       _            _         __  __                _                   
% |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
% | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
% |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
% |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
%
% Here are defined auxiliary functions used in the public members or for other simpler computations

    % Given a robot and an observation vector, it returns the associated landmark estimation (with
    % uncertainty) vector and place it in the map's landmark buffer.
    function add_to_buffer(map, robot, observation_vector)

        if map.buffer_i >= map.buffer_length    % reset index if out of bounds
            map.buffer_i = 1;
        end
        
        i = map.buffer_i;                       % short name
        landmark_vector = cell(1, length(observation_vector));  % will store landmarks
        
        for k = 1:length(observation_vector)
            observation = observation_vector(k);                % current observation
            landmark_vector{k} = Landmark(robot, observation);  % add landmark from obs. and robot
        end
        
        map.landmark_buffer{i} = landmark_vector;               % store in buffer
        map.buffer_i = map.buffer_i + 1;                        % update index
        
    end

    
    % Given the configuration set for the map, it initializes a grid
    function grid = initialize_grid(map)

        conf    = map.grid_configuration;
        Nx, Ny  = compute_grid_size(conf)
        grid    = zeros(map.buffer_length, Nx, Ny);

    end


    % It returns the list of the candidates that can be added to the map. It checks that a square in 
    % the grid is ammissible for all frames in buffer and compute the landmark position with a  
    % weighted least square using BLUE estimator.
    function candidates = find_candidates(map, grid, conf)

        [Nx, Ny] = compute_grid_size(conf);
        candidates = [];

        landmark_used = cell(1, map.buffer_length);
        for i = 1:map.buffer_length
            landmark_used{i} = logical(zeros(1, length(map.landmark_buffer{i})));
        end
        
        for i = 1:Nx % iterate over the grid
        for j = 1:Ny 

            if all(grid(:, i, j) ~= 0) % check if all occupancy grids have a compatible observation

                % Initialization for the BLUE estimator
                z = zeros(2*map.buffer_length, 1); % initialize the z vector
                H = zeros(2*map.buffer_length, 2); % initialize the H matrix
                R = zeros(2*map.buffer_length);    % initialize the R matrix

                landmark_list = cell(1, map.buffer_length); % initialize the landmarks to fuse
                for k = 1:map.buffer_length
                
                    landmark_list{k} = map.landmark_buffer{k}(grid(k, i, j));   % extract landmark
                    grid(grid(k, :, :) == grid(k, i, j)) = 0;                   % remove from grid
                    
                    % update the z, H and R matrices
                    z(2*k-1:2*k)            = landmark_list{k}.x;
                    H(2*k-1:2*k, :)         = eye(2);
                    R(2*k-1:2*k, 2*k-1:2*k) = landmark_list{k}.P;

                end

                % BLUE estimation
                new_candidate   = Landmark();
                new_candidate.x = (H'*inv(R)*H)\(H'*inv(R)*z);
                new_candidate.P = inv(H'*inv(R)*H);

                % add the landmark to the list of candidates
                candidates = [candidates; new_candidate];

            end
        end
        end
    end


    % Given a vector of candidate landmarks that can be added to the map, check their compatibility
    % with the current map and return the list of the new landmarks that can be added.
    function admissibles = check_candidates(map, candidates)
        
        admissibles = [];
        
        for i = 1:length(candidates)    % for each candidate
            
            landmark = candidates(i);   % current landmark to check
            insert_to_map = true;       % flag to check if the landmark can be added

            % If the covariance of the feature is too big, then it's not a good candidate as it do 
            % not have a good enough precision
            if max(abs(eig(landmark.P))) > map.grid_configuration.max_sigma    
                insert_to_map = false;
                continue;
            end

            for j = 1:map.size()        % check for an overall with the already present landmarks
                
                % If it's too probabile that the candidate landmark is the same as the one in the 
                % map, then just disregard the candidate
                if mvnpdf(landmark.x, map.landmark_vector(j).x, map.landmark_vector(j).P) ...
                        > 0.9
                    insert_to_map = false;
                    break;
                end

            end

            if insert_to_map
                admissibles = [admissibles; landmark];
            end
        end
    end
    
    
    % Given a landmark that can be added, initialize it properly in the map
    function add_landmark_to_map(map, landmark)
        %%  TODO

    end

end % methods
end % Laserscan class


%  ____       _            _          __                  _   _                 
% |  _ \ _ __(_)_   ____ _| |_ ___   / _|_   _ _ __   ___| |_(_) ___  _ __  ___ 
% | |_) | '__| \ \ / / _` | __/ _ \ | |_| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
% |  __/| |  | |\ V / (_| | ||  __/ |  _| |_| | | | | (__| |_| | (_) | | | \__ \
% |_|   |_|  |_| \_/ \__,_|\__\___| |_|  \__,_|_| |_|\___|\__|_|\___/|_| |_|___/

% It modifies the grid (with configuration "conf") based on
%   - index:        the index of the landmark vector in the map buffer
%   - landmarks:    the landmark vector in the map buffer 
function grid = update_grid(grid, index, landmarks, conf)

    for k = 1:length(landmarks)

        land = landmarks(k); % extract the current landmark

        [U, S, V] = svd(land.covariance); % compute svd on the landmark's covariance
        diag = U(:, 1) * sqrt(S(1, 1));   % extract the first eigenvector with associated length
        Delta_x = 3 * abs(diag(1));
        Delta_y = 3 * abs(diag(2));

        [i_min, j_min] = grid_cartesian_to_indexes(grid, land.x - Delta_x, land.y - Delta_y, conf) - 1;
        [i_max, j_max] = grid_cartesian_to_indexes(grid, land.x + Delta_x, land.y + Delta_y, conf);
        
        for i = i_min:i_max 
            for j = j_min:j_max
                pt                  = grid_indexes_to_cartesian(i, j, conf);
                point_in_grid       = (mvnpdf(pt, land.x, land.P) > conf.threshold);
                grid(index, i, j)   = k * point_in_grid;
            end
        end
    end
end


% Given the configuration of the map, it returns it's size in terms of number of cells along x and y
function [Nx, Ny] = compute_grid_size(conf)

    Nx = ceil((conf.UB_x - conf.LB_x) / conf.dx);
    Ny = ceil((conf.UB_y - conf.LB_y) / conf.dy);
end


% Given two cartesian coordianate [x, y] and a configuration for the grid, it returns the correct
% indexes to access the grid
function [i, j] = grid_cartesian_to_indexes(x, y, conf)

    i = ceil((x - conf.LB_x) / conf.dx);
    j = ceil((y - conf.LB_y) / conf.dy);

    [Nx, Ny] = compute_grid_size(conf);
    if i > Nx
        warning('Trying to access a cell outside the grid along the x direction');
        i = Nx;
    end
    if j > Ny
        warning('Trying to access a cell outside the grid along the y direction');
        j = Ny;
    end
end


% Given two indexes [i, j] of a grid with configuration conf, it return the respective cartesian
% coordinate [x, y]
function [x, y] = grid_indexes_to_cartesian(i, j, conf)

    x = conf.LB_x + (i - 1) * conf.dx;
    y = conf.LB_y + (j - 1) * conf.dy;
end