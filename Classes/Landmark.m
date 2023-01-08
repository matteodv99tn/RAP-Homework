classdef Landmark < handle 

%     _   _   _        _ _           _            
%    / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
%   / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
%  / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
% /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
%                                                 
properties 

    x;  % state estimate of the landmark (2x1 vector)
    P;  % related covariance (2x2 matrix)
                 
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

    % Constructor of the class takes as inputs the robot state and the observation and projects the observation,
    % which is in a local frame, into the absolute frame.
    % To project the uncertainty in the global frame we have to add to the covariance of the observation in the global frame,
    % the projection of the covariance of the robot into the global frame: 
    % global covariance = HOBS*R*HOBS + HR*P*HR'. 
    % Where:
    %   - R is the covariance of the observation
    %   - HOBS is the jacobian of the transformation w.r.t the observation
    %   - P is the covariance of the local observation.
    %   - HR is the jacobian of the transformation  w.r.t. the robot state
    
    function obj = Landmark(robot, observation) % constructor
        
        if nargin < 2
            obj.x = zeros(2, 2);
            obj.P = zeros(2, 2);
            return;
        end

        % robot state
        x_rob = robot.x(1);
        y_rob = robot.x(2);
        t_rob = robot.x(3);
        % observation vector
        xp = observation.z(1);
        yp = observation.z(2);

        % Transformation of reference frame from local (xp, yp) to global (x_land, y_land)
        x_land = x_rob + xp * cos(t_rob) - yp * sin(t_rob); 
        y_land = y_rob + xp * sin(t_rob) - yp * cos(t_rob);

        [Jg_x_robot, Jg_x_obs] = compute_jacobians(obj, robot,observation);
        
        obj.x = [x_land; y_land];
        obj.P = Jg_x_obs*observation.R*Jg_x_obs' + Jg_x_robot*robot.P*Jg_x_robot';

     
    end
    
    
    % Compute the jacobians of the observation function
    % Jg_x_robot: jacobian of the observation function with respect to the robot state
    % Jg_x_obs: jacobian of the observation function with respect to the landmark state
    % robot: robot object
    % obj: landmark object
    %
    % Observation function to which compute the jacobians:
    % x_land = x_rob + xp * cos(t_rob) - yp * sin(t_rob) 
    % y_land = y_rob + xp * sin(t_rob) + yp * cos(t_rob)
    function [Jg_x_robot, Jg_x_obs] = compute_jacobians(obj, robot, observation)
        
        x_rob   = robot.x(1);             % robot state
        y_rob   = robot.x(2);
        t_rob   = robot.x(3);
        xp      = observation.z(1);     % observation state
        yp      = observation.z(2);

        % Jacobian w.r.t the robot state
        j11 = 1;
        j12 = 0;
        j13 = - xp * sin(t_rob) - yp * cos(t_rob);
        j21 = 0;
        j22 = 1;
        j23 = xp * cos(t_rob) - yp * sin(t_rob);
        Jg_x_robot = [j11, j12, j13;
                      j21, j22, j23];
        
        % Jacobian w.r.t the observations
        j11 = cos(t_rob);
        j12 = - sin(t_rob);
        j21 = sin(t_rob);
        j22 = cos(t_rob);
        Jg_x_obs = [j11, j12;
                    j21, j22];

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