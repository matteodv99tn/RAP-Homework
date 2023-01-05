classdef Map < handle 

%     _   _   _        _ _           _            
%    / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
%   / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
%  / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
% /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
%                                                 
properties 

    landmark_vector;
    laserscan_buffer;
                 
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
    end

    % Given the map and a robot placed inside it that provides a vector of observations, this
    % function should be able to generate:
    %  - the innovation vector z;
    %  - the Jacobian of the observation function H_x;
    %  - the covariance matrix R of the measurement.
    function [z, H_x, R] = compute_innovation(map, robot, observation_vector)

        P1, P2 = obj.compute_correspondences(robot, observation_vector)

        %% TODO

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


    % Update
    function update_map(map, new_laserscan)
        %% TODO

    end

    function loop_closure(map, observation_vector)
        %% TODO

    end

%  ____       _            _         __  __                _                   
% |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
% | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
% |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
% |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
%
% Here are defined auxiliary functions used in the public members or for other simpler computations



end % methods
end % Laserscan class