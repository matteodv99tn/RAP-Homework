classdef Odometry < handle 

%     _   _   _        _ _           _            
%    / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
%   / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
%  / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
% /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
%                                                 
properties 
    dx;         % x increment
    dy;         % y increment
    dtheta;     % theta increment
    dX;         % column vector of the 3 increments
    Q;          % covariance matrix of the odometry
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

    % Take as input the 3 odometry data and set the properties of the class
    function obj = Odometry(data) % constructor
        obj.dx     = data(1);     
        obj.dy     = data(2);     
        obj.dtheta = data(3);
        obj.dX     = [obj.dx; obj.dy; obj.dtheta];

        obj.Q      = 1e-4 * eye(3); 
    end


    % Given the current state X_curr of the robot, it computes the updated state X_next based on the
    % odometry dX. It's also able to return the jacobian of such transformation.
    % Based on the update function X_next = f(X_curr, dX) described as
    %       x_next     = x_curr + cos(theta_curr) * dx - sin(theta_curr) * dy + noise_x
    %       y_next     = y_curr + sin(theta_curr) * dx + cos(theta_curr) * dy + noise_y
    %       theta_next = theta_curr + dtheta + noise_theta
    % that can be rewritten in a form X_next = X_curr + A*dX, where A is actually the jacobian of 
    % the transformation with respect to the states.
    function [X_next, jac] = update_states(obj, X_curr) 

        theta  = X_curr(3);
        jac    = [ cos(theta), -sin(theta), 0; ...
                   sin(theta),  cos(theta), 0; ...
                   0,           0,          1  ];
        X_next = X_curr + jac*obj.dX;

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