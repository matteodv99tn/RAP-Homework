classdef Laserscan 

%     _   _   _        _ _           _            
%    / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
%   / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
%  / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
% /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
%                                                 
properties 
    polar;                      % Nx2 matrix containing, for each row, the polar coordinate of the
                                % measure [m] and the corresponding measurement angle [rad], so in a
                                % form 
                                %       [r, theta]
                                % N is the number of measurement in the scan
    cartesian;                  % Nx2 matrix that's the transformation in cartesian coordinates of 
                                % the measurement; is row is of the type
                                %       [x, y]          both values expressed in [m]
    features;              
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

    % Constructor of the class; the inputs that must be provided are
    %   - measures:     vector containing the polar measurements;
    %   - angles:       vector (of the same length) containing the corresponing angle at which the 
    %                   the measure have been collected
    % Note: vectors can be either be column or row.
    function obj = Laserscan(measures, angles)
        
        obj.polar       = [ to_column_vector(measures), to_column_vector(angles) ];
        obj.cartesian   = [ cos(obj.polar(:, 2)), sin(obj.polar(:, 2))] .* obj.polar(:, 1);
        obj.features    = [];

        function out = to_column_vector(in) % converts any vector into a column vector
            if isrow(in)
                out = in';
            else
                out = in;
            end
        end
    end

    
    % Overloading of the plot function
    function plot(obj)

        plot(obj.cartesian(:, 1), obj.cartesian(:, 2), '.k', ...
            'DisplayName', 'point cloud');
        axis equal;
        hold on;
        grid on;

    end

    % Main function that needs to be called for extracting the feature of the laserscan
    function feat = extract_feature(obj)

        segments        = obj.seeding();        
        segments        = obj.segment_reduction(segments); 
        segments        = obj.expand_seeds(segments);  
        feat            = segments;
        obj.features    = feat;
    end


%  ____       _            _         __  __                _                   
% |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
% | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
% |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
% |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
%
% Here are defined auxiliary functions used in the public members or for other simpler computations

    % Based on the paper:
    % https://journals.sagepub.com/doi/pdf/10.1177/1729881418755245
    % this algorithm searches for seed segments inside the laserscan; to achieve the result the 
    % following parameters are used:
    %   - N_min:        minimum number of points that are required to generate a line
    %   - N_back:       if the seeding succeed, the algorithms starts searching for new seed 
    %                   segments from this number of points before the yet found endpoint 
    % Given a starting index, the algorithm computes the least square fitting line among the N_min
    % cartesian points; the segment is regarded as a seed if:
    %   - the distance between start and end points is bounded by the mean polar measure (to avoid)
    %     undesired failure;
    %   - if all points are restrained in a region of distance "epsilon" from the fitted line
    % The result is a Nx2 matrix (with N the number of seed segments generated) that for each row 
    % contains the start and endpoint index of the generated valid segment
    function seeds = seeding(obj)
        
        % Parameters initialization
        N_min   = 8;
        N_back  = 2;

        seeds   = [];   
        N       = size(obj.cartesian, 1);

        i       = 1;
        j       = i + N_min;

        while j < N

            line = obj.fit_line(i, j);                      % least square fitting

            if obj.check_line_correctness(line, i, j, 0.05, 1)  % check line
                seeds(end+1, :) = [i, j, line];
                i = j - N_back;
            else
                i = i + 1;
            end

            j = i + N_min;
        end
    end

    
    % Given two seeds (seed_a, seed_b), the function tries to understand if it possible to merge 
    % them in a single line; seeds are vectors of the form
    %           [start, end, a, b, c]
    % where
    %   - "start" and "end" are respectively the starting/ending indexes of the segment inside the
    %     the laserscan;
    %   - "a, b, c" are the coefficients of the line expressed in the form 
    %           a*x + b*y + c = 0
    % The seeds are merged if all the following conditions are met:
    %   - there is an overlap in the two segments;
    %   - the relative orientation of the two segments is lower then a critic angle "alpha_critic";
    %   - it is possible to correctly fit a line with all points
    function res_seeds = join_seeds(obj, seed_a, seed_b)

        alpha_critic = 15;   % [deg]

        v1 = seed_a(3:4);   % extract the normal direction of seed_a
        v2 = seed_b(3:4);   % extract the normal direction of seed_b

        res_seeds = [seed_a; seed_b];       % default return value
        
        if seed_b(1) > seed_a(2)            % check overlap condition: if not overlap -> return
            return;
        end

        confr = (v1 * v2') / (norm(v1) * norm(v2));
        if abs(confr) < cos(alpha_critic * pi / 180)   % check relative orientation
            return;
        end      
        
        line = obj.fit_line(seed_a(1), seed_b(2));      % check feasibility of the line
        if obj.check_line_correctness(line, seed_a(1), seed_b(2), 0.05, 0)
            res_seeds = [seed_a(1), seed_b(2), line];
        end  

    end
    
    % Given a vector of segments, function tries to reduce the number of
    % features, joining as many segments as possible in a single line
    % Function returns the new vector of features, reduced
    function features_old = segment_reduction(obj, segments)
        features_old = segments;
        dim_now = 0;
        dim_prev = 1;
        iter = 0;
        while(dim_now ~= dim_prev)
            iter = iter + 1;
            k = 1;
            dim_prev = size(features_old, 1);
            while k < (size(features_old, 1))
                % We check if we can join two consecutive segments
                % size(reduced_feature) = 1 --> segments joint
                % size(reduced_feature) = 2 --> we can't join the segments
                % -> situation remains the same as before and we continue
                reduced_feature = obj.join_seeds(features_old(k,:), features_old(k+1,:));
                if size(reduced_feature, 1) == 1
                    % we can reduce the vector of feature by saving in position k the new
                    % join segments and deleting the k+1 row
                    features_old(k,:) = reduced_feature;
                    features_old(k+1,:) = [];
                end
                k = k + 1;
            end
            dim_now = size(features_old, 1);
        end
    end

    
    function features = expand_seeds(obj, seeds)

        features = zeros(size(seeds));

        for n = 1:size(features, 1) % for each feature

            i = seeds(n, 1);
            j = seeds(n, 2);
            line = seeds(n, 3:5);   % extract the line coefficients
            
            while line_compliant(obj, line, i) && i > 1
                i = i - 1;
            end
            i = i + 1;

            while line_compliant(obj, line, j) && j < size(obj.cartesian, 1)
                j = j + 1;
            end
            j = j - 1;

            features(n, :) = [i, j, obj.fit_line(i, j)];

        end

        function is_compliant = line_compliant(obj, line, index_to_check)
                
            P_predicted = predict_point(line, obj.polar(index_to_check, 2))

            if point_point_distance(P_predicted, obj.cartesian(index_to_check, :)) < 0.05 && ...
               line_point_distance(line, obj.cartesian(index_to_check, :)) < 0.05
                is_compliant = true;
            else
                is_compliant = false;
            end
        end

    end

    % Given a line (vector of coefficients [a, b, c] for a line of the type a*x + b*y + c = 0) and 
    % star/end indexes i, j, it checks that:
    %   - the distance between start/end points it's not too high w.r.t. the mean polar measurement;
    %   - each point is restrained in a region of distance "epsilon" from the line
    % epsilon is given as an input of this function because it is used in
    % different moments of the main alghoritm --> for the first seed
    % detenction it's necessary to use a little "epsilon", later when we need
    % to grow the lines it's ok to use a bigger "epsilon"
    % ppdist is flag used to avoid the check on the point to point distance
    % when the segment are joined. (A long wall may have very far points)
    function is_line = check_line_correctness(obj, line, i, j, epsilon, ppdist)
    
        is_line = true;         % by default we assume that the segment is a proper seed for the
                                % proposed indexes
        
        mean_radius = mean(obj.polar([i,j], 1));
        if ppdist == 1 % check on the distant point to point only when I'm generating the seeds
            if obj.point_point_distance(i, j) > 0.3 * mean_radius 
                is_line = false;
                return;
            end
        end

        for k = i:j % check that the distance from each point from the line is below threshold
            
            if obj.line_point_distance(line, k) > epsilon 
                is_line = false;
                return
            end

            P_curr = obj.cartesian(k, :);
            P_pred = predict_point(line, obj.polar(k, 2));
            if point_point_distance(P_curr, P_pred) > 0.09
                is_line = false;
                return
            end
        end
    end

    % Given start (i) and endpoint index (j), it computes the parameters [a, b, c] of the line in 
    % the form
    %       a*x + b*y + c = 0
    % that's the least square fit
    function coeff = fit_line(obj, i, j)

        X = obj.cartesian(i:j, 1);  % set of x measurements in the segment
        Y = obj.cartesian(i:j, 2);  % set of y measurements in the segment
        I = ones(length(X), 1);     % vector of ones with same dimensions as the number of points

        % Note: the matrix of the regressor A and of the output b is chosen accordingly to the 
        % "spreadness" of the data in order to improve the numerical stability of the algorithm
        if std(X) > std(Y)
            A = [I, X];
            b = Y;
            X = pinv(A) * b;
            coeff = [X(2), -1, X(1)];
        else
            A = [I, Y];
            b = X;
            X = pinv(A) * b;
            coeff = [-1, X(2), X(1)];
        end
    end


    % overload of the point_point_distance function to work just with laserscan indexes
    function res = point_point_distance(obj, i, j)
        res = point_point_distance(obj.cartesian(i, :), obj.cartesian(j, :));
    end


    % overload of the line_point_distance function to work with laserscan point index
    function res = line_point_distance(obj, line, i)
        res = line_point_distance(line, obj.cartesian(i, :));
    end
    
    
    % overload of the predict_point function to work with laserscan index
    function res = predict_point(obj, line, i)
        res = predict_point(line, obj.polar(i, 2));
    end

    end % methods
end % Laserscan class