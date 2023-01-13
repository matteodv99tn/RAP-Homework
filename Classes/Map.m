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
    grid;
    distances;

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
        
        obj.buffer_length   = 20;
        %obj.landmark_buffer = cell(1, obj.buffer_length);
        obj.landmark_buffer = cell(1, 0);
        obj.distances = cell(1, 0);
        obj.buffer_i        = 1;
        obj.grid_configuration = struct( ...
                'LB_x',         -25, ...   % lower bound for the x coordinate of the map
                'UB_x',         25, ...    % upper bound for the x coordinate of the map
                'LB_y',         -25, ...   % lower bound for the y coordinate of the map
                'UB_y',         25, ...    % upper bound for the y coordinate of the map
                'dx',           0.15, ...    % grid step size w.r.t. x
                'dy',           0.15, ...    % grid step size w.r.t. y
                'threshold',    0.2,...     % threshold for the occupancy grid
                'max_sigma',    0.4 ...     % maximum value for the standard deviation of the 
                                    ...     % landmarks to be considered valid
            );

        obj.grid = obj.initialize_grid();

    end


    function plot(map, n_new)
        clf;
        if nargin < 2
            n_new = 0;
        end

        for i = 1:map.size()
            
            plot(map.landmark_vector(i).x(1), map.landmark_vector(i).x(2), '*b');
            hold on;
            plotErrorEllipse([map.landmark_vector(i).x(1),map.landmark_vector(i).x(2)], map.landmark_vector(i).P, 0.95,'b')
            hold on;
        end

        buffer_lands = map.landmark_buffer{map.buffer_i-1};
        for i = 1:length(buffer_lands)-n_new

            plot(buffer_lands{i}.x(1), buffer_lands{i}.x(2), '+r');
            plotErrorEllipse([buffer_lands{i}.x(1),buffer_lands{i}.x(2)], buffer_lands{i}.P, 0.95,'r')
            hold on;

        end
        for i = length(buffer_lands)-n_new+1:length(buffer_lands)

            plot(buffer_lands{i}.x(1), buffer_lands{i}.x(2), 'om');
            plotErrorEllipse([buffer_lands{i}.x(1),buffer_lands{i}.x(2)], buffer_lands{i}.P, 0.95,'m')
            hold on;

        end

        
    end

    
    % Given the map and a robot placed inside it that provides a vector of observations, this
    % function should be able to generate:
    %  - the innovation vector z;
    %  - the Jacobian of the observation function H_x;
    %  - the covariance matrix R of the measurement.
    % Reference to equation taken from:
    % https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf
    function [z, H_x, R] = compute_innovation(map, robot, observation_vector)

        [P1, P2] = map.compute_correspondences(robot, observation_vector);

        % Initialization:
        dim_z   = 2 * length(P1);       % dimension of the observation vector
        dim_P   = 3 + 2*map.size();     % dimension of the P matrix (robot pose + (2x)landmarks)

        z       = zeros(dim_z, 1);
        H_x     = zeros(dim_z, dim_P);
        R       = zeros(dim_z, dim_z);

        for k = 1:length(P1)

            i_obs       = P1(k);    % index of the measurement inside the observation vector
            i_land      = P2(k);    % index of the landmark inside the map
            observation = observation_vector{i_obs};    % current observation of interest
            landmark    = map.landmark_vector(i_land);  % current landmark of interest

            % Compute the estimated observation (with jacobian) bases on the current robot pose 
            % estimation and the landmark position estimation
            [h, Jh_x_rob, Jh_x_land] = robot.landmark_to_observation(landmark);

            % Fill the innovation vector, the Jacobian and the covariance matrix
            z(2*k-1:2*k)                            = observation.z - h;        % eq (19, 26)
            H_x(2*k-1:2*k, 1:3)                     = Jh_x_rob;                 % eq (25)
            H_x(2*k-1:2*k, 3+2*i_land-1:3+2*i_land) = Jh_x_land;                % eq (25)
            R(2*k-1:2*k, 2*k-1:2*k)                 = observation.R;

            if ~all(eig(observation.R) > 0)
                fprintf('In compute_innovation function');
                error('The covariance matrix of the observation is not positive definite');
            end

        end
    end


    % The objective of this function is to compute the correspondence between the landmarks 
    % (contained in the map object itself --> all the global landmarks) and the observations 
    % coming from a robot (landmarks of a single scan from the pov of the robot).
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
    % The correspondence is computed by comparing the distribution of the observations and the
    % landmarks in the map, taking in account of the covariances.
    % If 2 observations are associated to the same landmark, the best association is chosen
    function [P1, P2] = compute_correspondences(map, robot, observation_vector)
        is_closed = false;
        % Initialization:
        P1 = [];
        P2 = [];

        landmarks = map.landmark_vector;
        
        if map.size() > 2*length(observation_vector)
            [Pone,Ptwo,is_closed] = loop_closure(map, observation_vector, robot);
            P1 = Pone;
            P2 = Ptwo;
        end

        if is_closed == false
            for i = 1:length(observation_vector)
                absolute_observation = Landmark(robot, observation_vector{i});
                % absolute_observation = robot.observation_to_landmark(observation_vector(i));
                for j = 1:length(landmarks)
                    % pdf_land_j = mvnpdf(absolute_observation.x, landmarks(j).x, landmarks(j).P);
                    % Compute mahalanobis distance between the observation and the landmark
                    % (the mahalanobis distance is a measure of the distance between two multivariate
                    % normal distributions)
                    d_obs_land = mahalanobis_distance(absolute_observation.x, landmarks(j).x, landmarks(j).P);
                    if d_obs_land < 2
                        index = find(P2==j);
                        if length(index) > 0
                            % If the landmark is already associated to another observation, then
                            % we choose the best association (the one with the highest probability)
                            X1 = absolute_observation.x;
                            X2 = landmarks(P2(index)).x;
                            X3 = landmarks(P2(index)).P;
                            d_obs_land_2 = mahalanobis_distance(X1, X2, X3);
                            if d_obs_land_2 < d_obs_land
                                continue;
                            else
                                P1(index) = [];
                                P2(index) = [];
    
                                disp('WARNING -> two observation are associated to the same landmark');
    
                            end
                        end
                        P1 = [P1; i];
                        P2 = [P2; j];
                    end
                end
            end
    
            if(length(P1) ~= length(P2))
                error([ 'The number of correspondences is not the same for P1 and P2,',
                        ' could not compute the innovation vector']);
            end
        end
    
    end


    % The goal of this function is to add the new observation to a buffer and check if it's possible
    % to add a new landmark to the map. 
    function new_landmarks = update_map(map, robot, observation_vector)

        map.add_to_buffer(robot, observation_vector);
        grids = map.grid;

        grids = update_grid(grids, map.buffer_i - 1, map.landmark_buffer{map.buffer_i - 1}, map.grid_configuration);
        
        % tmp = zeros(size(grids, 2), size(grids, 3));
        % tmp(:,:) = grids(map.buffer_i-1,:,:);
        % figure(1);
        % heatmap(tmp);
        % grid off;

        candidates      = map.find_candidates(grids, map.grid_configuration);
        new_landmarks   = map.check_candidates(candidates, robot);

        for i = 1:length(new_landmarks)
            map.landmark_vector = [map.landmark_vector; Landmark(robot, observation_vector{new_landmarks(i)})];
        end

        map.grid = grids;
    
    end
    
    % When I see a new observation, I project it into landmark.
    % Because of the drift the set of new landmark doesn't overlapp the old features
    % I can compute the centroids of the new features and 

    % CHIAMARE LA FUNZIONE CONTROLLANDO LA LUNGHEZZA DELL MAPPA > 2*lenght(observation_vector)+1
    function [P1,P2,is_closed] = loop_closure(map, observation_vector, robot)
        P1 = [];
        P2 = [];
        is_closed = false;
        landmarks = map.landmark_vector;
        sumx = 0;
        sumy = 0;
        absolute_obs_vector = cell(1,length(observation_vector));
        x_obs = zeros(1,length(observation_vector));
        y_obs = x_obs;

        for i = 1:length(observation_vector)
            absolute_obs_vector{i} = Landmark(robot,observation_vector{i});
            sumx = sumx + absolute_obs_vector{i}.x(1);
            sumy = sumy + absolute_obs_vector{i}.x(2);
        end
        % Computing the centroid of the observations
        centroid_obs = [sumx;sumy]./length(observation_vector);
        % Finding the landmarks inside the rectangle
        [~,~,~,~,maxx,maxy] = compute_boundaries(map,robot,observation_vector);
        land_inside = old_landmark_inside_rectangle(map,robot,observation_vector,maxx,maxy);

        sumx = 0;
        sumy = sumx;
        % Controlling if there are landmark in the controlling area
        if length(land_inside) > floor(length(observation_vector)*0.7)
            for i = 1:length(land_inside)
                sumx = sumx + map.landmark_vector(land_inside(i)).x(1);
                sumy = sumy + map.landmark_vector(land_inside(i)).x(2);
            end
        else
            fprintf('NOT ENOUGH LANDMARK IN THE AREA\n');
            return;
        end
        % Computing the centroid landmark
        centroid_land = [sumx;sumy]./length(land_inside);

        delta_centroids = centroid_land - centroid_obs;
        if point_point_distance(centroid_obs,centroid_land) > 3
            fprintf('CENTROIDS TOO DISTANT\n');
            return;
        end
        % Defining the vector of possibile rotation that we will try to find the correspondence
        theta_test = linspace(-20,20,7);
        % Defining the vector of possible displacement in x and y 
        displacement_x = -0.7 + (0.7+0.7)*rand(1,8);
        displacement_y = -0.7 + (0.7+0.7)*rand(1,8);
        
%             
        % For each displacement in x
        for z = 1:length(displacement_x)
            % For each displacement in y
            for l = 1:length(displacement_y)
                % For each angle
                for j = 1:length(theta_test)
                    % For each observations
                    % figure(3),clf;
                    for i = 1:length(absolute_obs_vector) 
                        absolute_obs_vector{i}.x(1) = absolute_obs_vector{i}.x(1) + displacement_x(z); 
                        absolute_obs_vector{i}.x(2) = absolute_obs_vector{i}.x(2) + displacement_y(l); 
                        % Translating and rotating the observations into the landmarks
                        [transformx,transformy] = rototrasl(map,centroid_obs,absolute_obs_vector{i},theta_test(j),delta_centroids);
                        % For each landmark in the rectangle
                        for k = 1:length(land_inside)
                            
                            d_obs_land = mahalanobis_distance([transformx,transformy]', landmarks(land_inside(k)).x, landmarks(land_inside(k)).P);
                            if d_obs_land < 3

                                index = find(P2==land_inside(k));
                                if length(index) > 0
                                    % If the landmark is already associated to another observation, then
                                    % we choose the best association (the one with the highest probability)
                                    X1 = [transformx,transformy]';
                                    X2 = landmarks(P2(index)).x;
                                    X3 = landmarks(P2(index)).P;
                                    d_obs_land_2 = mahalanobis_distance(X1, X2, X3);
                                    if d_obs_land_2 < d_obs_land
                                        continue;
                                    else
                                        P1(index) = [];
                                        P2(index) = [];
            
                                        disp('WARNING -> two observation are associated to the same landmark');
            
                                    end
                                end
                                P1 = [P1; i];
                                P2 = [P2; land_inside(k)];
                                
                            end
                        end
                    end

                    if(length(P1) ~= length(P2))
                        error([ 'The number of correspondences is not the same for P1 and P2,',
                                ' could not compute the innovation vector']);
                    end
                    
                    if length(P1) >= floor(length(observation_vector)*0.5)
                        is_closed = true;
                        fprintf('DETECTED LOOP\n');


                        
                        figure(3),clf;
                        plot(centroid_land(1),centroid_land(2),'ok');
                        hold on;
                        plot(centroid_obs(1),centroid_obs(2),'*r');
                        hold on
                        for i = 1:length(absolute_obs_vector)
                            plot(absolute_obs_vector{i}.x(1),absolute_obs_vector{i}.x(2),'or')
                            hold on
                            plot(absolute_obs_vector{i}.x(1)+delta_centroids(1),absolute_obs_vector{i}.x(2)+delta_centroids(2),'og');
                            hold on
                            [transformx,transformy] = rototrasl(map,centroid_obs,absolute_obs_vector{i},theta_test(j),delta_centroids);
                            plot(transformx,transformy,'og');
                        end
                        hold on
                        for i = 1:length(land_inside)
                            plot(map.landmark_vector(land_inside(i)).x(1), map.landmark_vector(land_inside(i)).x(2), '^k');
                            axis equal
                            hold on;
                        end

                        



            
                        pause();
                        return;
                    else
                        P1 = [];
                        P2 = P1;
                        fprintf('LOW CORRESPONDENCE\n');
                    end

                end
            end
        end

        

        
    end

    % Function that computes a rototranslation of the observations from the actual reference frame to the ones
    % of the old landmarks
    function [transformx,transformy] = rototrasl(map,centroid_obs,abs_observation,theta,delta_centroids)

        abs_observation.x(1) = abs_observation.x(1) + delta_centroids(1);
        abs_observation.x(2) = abs_observation.x(2) + delta_centroids(2);

        origin_new_x = centroid_obs(1) + delta_centroids(1); 
        origin_new_y = centroid_obs(2) + delta_centroids(2);
        
        dist_point_centroid = point_point_distance(abs_observation.x,[origin_new_x,origin_new_y]);
        angle_observations = atan2(abs_observation.x(2),abs_observation.x(1)); % alpha
        
        abs_observation.x(1) = dist_point_centroid*cos(angle_observations + theta);
        abs_observation.x(2) = dist_point_centroid*sin(angle_observations + theta);

        transformx = abs_observation.x(1);
        transformy = abs_observation.x(2);
    end



    % Given a map it returns the number of landmarks, i.e. the length of the landmark vector
    function dim = size(map)
        dim = length(map.landmark_vector);
    end


    function [A,B,C,D,maxx,maxy] = compute_boundaries(map,robot,observation_vector)
        xr = robot.x(1);
        yr = robot.x(2);
        tr = robot.x(3);
        maxx = 0;
        maxy = 0;
        MAXX = 18;
        MINX = 5;
        MAXY = 7;
        MINY = 3;
        % Selecting the control area
        for i = 1:length(observation_vector)
           if observation_vector{i}.z(1) > maxx
                maxx = observation_vector{i}.z(1);
            end
           if abs(observation_vector{i}.z(2)) > maxy
                maxy = abs(observation_vector{i}.z(2));
            end
        end

        if maxx > MAXX
                maxx = MAXX;
            else if maxx < MINX
                maxx = MINX;
            end
        end
        if maxy > MAXY
                maxy = MAXY;
            else if maxy < MINY
                maxy = MINY;
            end
        end

        A = [xr + maxy*sin(tr);yr - maxy*cos(tr)];
        D = [xr - maxy*sin(tr);yr + maxy*cos(tr)];
        B = [A(1) + maxx*cos(tr),A(2) + maxx*sin(tr)];
        C = [D(1) + maxx*cos(tr),D(2) + maxx*sin(tr)];

    end
    % Given the map and the robot gives the landmark near the robot
    function land_inside = old_landmark_inside_rectangle(map,robot,observation_vector,radiuspar,radiusper)

        element_deleted = 2*length(observation_vector);

        land_inside = [];
        xr = robot.x(1);
        yr = robot.x(2);
        tr = robot.x(3);

        for i = 1:map.size() - element_deleted + 1
            [land_local,~,~] = landmark_to_observation(robot, map.landmark_vector(i));
            if land_local(1) > 0 && land_local(1) < radiuspar
                if land_local(2) > -radiusper && land_local(2) < radiusper
                    land_inside = [land_inside;i];
                end
            end
        
            

        end

    end

    


    % Alternative to the grid map.
    % This function create a buffer of distances.
    % If from the first to the last element of the buffer
    % there exists a chain of small distances then a new feature is added in the map
    function nw_land = up_map(map,robot, observation_new)
        nw_land = []; 
        if length(observation_new) > 0
            buffer_max = map.buffer_length;
            tmp = map.landmark_buffer;
            tmp1 = map.distances;
    
            % create the vector of landmark from observations
            for i = 1:length(observation_new)
                landmark_observed(i) = Landmark(robot, observation_new{i}); % vector
            end
    
            % shifting the buffer of landmark
            for i = 1:length(tmp)
                map.landmark_buffer{i+1} = tmp{i};  
            end
            map.landmark_buffer{1} = landmark_observed;
    
            % shifting the buffer of distances
            if length(map.landmark_buffer) > 1
                for i = 1:length(tmp1)
                    map.distances{i+1} = tmp1{i};
                end
            end
            map.distances{1} = [];
    
            
            
            % creating the matrix of distances
            % new element: | 1| 2| 3| 4|
            % previous element: | 1| 2|
            % in the matrix distances of 1 there are 4 raws and 2 colums
            % d11 d12
            % d21 d22
            % d31 d32
            % d41 d42
            % When the distances are computed the index of the most probable connection is stored for each raw
            % So I can reconstruct a chain of connection
            % I save the index in the last column of the matrix with a minus (easy to extract)
            if length(map.landmark_buffer) > 1
                for i = 1:size(map.landmark_buffer{1},2)
                    for j = 1:(size(map.landmark_buffer{2},2))
                        map.distances{1}(i,j) = mahalanobis_distance(map.landmark_buffer{1}(i).x,map.landmark_buffer{2}(j).x,map.landmark_buffer{1}(i).P);
                       
                    end
                    [minn,indexmin] = min(map.distances{1}(i,1:length(map.landmark_buffer{2})));
                    
                    
    
                    if minn < 0.4 % treshold of distance
                        map.distances{1}(i,length(map.landmark_buffer{2})+1) = -indexmin;
                
                    else
                        map.distances{1}(i,length(map.landmark_buffer{2})+1) = 0;
                    end
    
                    
                end
                    
            end
          
            % for each row of the distances of the first layer I have to
            % check if there is a chain up to the last layer simply following
            % the most probable connection stored before in the index
            raw = 1;
            if length(map.landmark_buffer) > buffer_max
                
                for j = 1:size(map.distances{1},1)
                    for i = 1:buffer_max-1
                        % If the element is negative I can pass to the second layer
                        if raw > size(map.distances{1},1)
                            break;
                        end
                        if map.distances{i}(raw,end) < 0
                            % The raw of the next layer is the index stored in the matrix distances
                            raw = -map.distances{i}(raw,end);
                            
                            if raw == 0 % If raw == 0 => no connection
                                break;
                            end
                            if i == buffer_max - 1  
                                if mahalanobis_distance(map.landmark_buffer{1}(j).x,map.landmark_buffer{buffer_max}(raw).x,map.landmark_buffer{1}(j).P) < 0.5                    
                                    nw_land = [nw_land,j]; % If I have a connection up to the last layer I add to the map
                                end

                                raw = j + 1;
                            end
    
                        else
                            
                            break;
                            raw = j + 1; % If no connection I restart from the next raw of the first layer
                        end
                        
                    end
                end
            end
            % Deleting the element outside the buffer
            if length(map.landmark_buffer) > buffer_max
                map.landmark_buffer(end) = [];
                map.distances(end) = [];
            end
            % Controlling to insert really new features
            tmp3 = 0;
            landmark_observed1 = landmark_observed(nw_land);
    
            for i = 1:map.size()
                for j = 1:length(nw_land)
                    if mahalanobis_distance(map.landmark_vector(i).x,landmark_observed1(j).x,map.landmark_vector(i).P) < 0.2 
                        tmp3(j) = nw_land(j);
                    end
                end
            end
            for i = 1:length(tmp3)
                irem = nw_land == tmp3(i);
                nw_land(irem) = [];
            end
            irem = nw_land == 0;
            nw_land(irem) = [];
            
            map.landmark_vector = [map.landmark_vector; landmark_observed(nw_land)'];

        end
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

        if map.buffer_i > map.buffer_length    % reset index if out of bounds
            map.buffer_i = 1;
        end
        
        i = map.buffer_i;                       % short name
        landmark_vector = cell(1, length(observation_vector));  % will store landmarks
        
        for k = 1:length(observation_vector)
            observation = observation_vector{k};                % current observation
            landmark_vector{k} = Landmark(robot, observation);  % add landmark from obs. and robot
        end
        
        map.landmark_buffer{i} = landmark_vector;               % store in buffer
        map.buffer_i = map.buffer_i + 1;                        % update index
        
    end

    
    % Given the configuration set for the map, it initializes a grid
    function grid = initialize_grid(map)

        conf    = map.grid_configuration;
        [Nx, Ny]  = compute_grid_size(conf);
        grid    = zeros(map.buffer_length, Nx, Ny);

    end


    % It returns the list of the candidates that can be added to the map. It checks that a square in 
    % the grid is ammissible for all frames in buffer and compute the landmark position with a  
    % weighted least square using BLUE estimator.
    function candidates = find_candidates(map, grid, conf)

        [Nx, Ny] = compute_grid_size(conf);
        candidates = [];
        
        for i = 1:Nx % iterate over the grid
            for j = 1:Ny 

                obs_index = 0;

                if all(grid(:, i, j) ~= 0) % check if all occupancy grids have a compatible observation

                    obs_index = grid(map.buffer_i - 1, i, j);

                    for k = 1:map.buffer_length
                        sel1            = grid(k, :, :) == grid(k, i, j);
                        subgrid         = grid(k, :, :);
                        subgrid(sel1)   = 0;
                        grid(k, :, :)   = subgrid;
                    end
                    
                    % add the landmark to the list of candidates
                    candidates = [candidates; obs_index];

                end
            end
        end
    end


    % Given a vector of candidate landmarks that can be added to the map, check their compatibility
    % with the current map and return the list of the new landmarks that can be added.
    function admissibles = check_candidates(map, candidates, robot)
        
        admissibles = [];
        
        for i = 1:length(candidates)    % for each candidate
            
            landmark = map.landmark_buffer{map.buffer_i - 1}{candidates(i)}; % current landmark to check
               
            insert_to_map = true;       % flag to check if the landmark can be added

            % If the covariance of the feature is too big, then it's not a good candidate as it do 
            % not have a good enough precision
            if max(abs(eig(landmark.P))) > map.grid_configuration.max_sigma    
                insert_to_map = false;
                continue;
            end

            for j = 1:map.size()        % check for an overall with the already present landmarks
                
                if norm(landmark.x - map.landmark_vector(j).x) < 0.2*0.2
                    insert_to_map = false;
                    continue;
                end

                % If it's too probabile that the candidate landmark is the same as the one in the 
                % map, then just disregard the candidate
                % if mvnpdf(landmark.x, map.landmark_vector(j).x, map.landmark_vector(j).P) ...
                %         > 0.99
                if mahalanobis_distance(map.landmark_vector(j).x, landmark.x, landmark.P) ...
                        < 3
                    insert_to_map = false;
                    continue;
                end

            end

            if insert_to_map
                admissibles = [admissibles; candidates(i)];
            end
        end
    end
    
    
    % % Given a landmark that can be added, initialize it properly in the map
    % function add_landmark_to_map(map, landmark)
    %     %%  TODO

    % end



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

    grid(index, :, :) = zeros(size(grid, 2), size(grid, 3));

    for k = 1:length(landmarks)

        land = landmarks{k}; % extract the current landmark

        [U, S, V] = svd(land.P); % compute svd on the landmark's covariance
        diag = U(:, 1) * sqrt(S(1, 1));   % extract the first eigenvector with associated length
        Delta_x = 3 * abs(diag(1));
        Delta_y = 3 * abs(diag(2));

        [i_min, j_min] = grid_cartesian_to_indexes( land.x(1) - Delta_x, land.x(2) - Delta_y, conf);
        
        i_min = i_min - 1;
        j_min = j_min - 1;
        [i_max, j_max] = grid_cartesian_to_indexes( land.x(1) + Delta_x, land.x(2) + Delta_y, conf);
        
        for i = i_min:i_max 
            for j = j_min:j_max
                [a,b]                  = grid_indexes_to_cartesian(i, j, conf);
                point_in_grid          = mahalanobis_distance([a; b], land.x, land.P) < 3;
                grid(index, i, j)      = k * point_in_grid;
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

    if i < 1
        warning('Trying to access a cell outside the grid along the x direction');
        i = 1;
    end
    if j < 1
        warning('Trying to access a cell outside the grid along the y direction');
        j = 1;
    end
end


% Given two indexes [i, j] of a grid with configuration conf, it return the respective cartesian
% coordinate [x, y]
function [x, y] = grid_indexes_to_cartesian(i, j, conf)

    x = conf.LB_x + (i - 1) * conf.dx;
    y = conf.LB_y + (j - 1) * conf.dy;
end


% function that computes the mahalanobis distance
function d = mahalanobis_distance(x, y, P)
    d = sqrt((x - y)' * inv(P) * (x - y));  % eq (83)
end

% Plot ellipsoid
function plotErrorEllipse(mu, Sigma, p, color)

    s = -2 * log(1 - p);

    [V, D] = eig(Sigma * s);

    t = linspace(0, 2 * pi);
    a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];
 
    plot(a(1, :) + mu(1), a(2, :) + mu(2),color);
end
