classdef Robot < handle 

%     _   _   _        _ _           _            
%    / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
%   / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
%  / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
% /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
%                                                 
properties
    x;          % current estimated position of the robot. Is a 3x1 vector of the form
                %       [x; y; theta]       x, y [m], theta [rad]
    P;          % 3x3 covariance matrix of the robot position estimate.
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

    function obj = Robot() % constructor
        x = zeros(3, 1);
        P = zeros(3, 3);
    end
    

    % Given a odometry measurement, update the robot position estimate and it's covariance
    % Based on equations (9) and (10) of the paper
    % https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf
    
    % Given the current state X_curr of the robot, it computes the updated state X_next based on the
    % odometry dX. It's also able to return the jacobian of such transformation.
    % Based on the update function X_next = f(X_curr, dX) described as
    %       x_next     = x_curr + cos(theta_curr) * dx - sin(theta_curr) * dy + noise_x
    %       y_next     = y_curr + sin(theta_curr) * dx + cos(theta_curr) * dy + noise_y
    %       theta_next = theta_curr + dtheta + noise_theta
    % that can be rewritten in a form X_next = X_curr + A*dX, where A is actually the jacobian of 
    % the transformation with respect to the states.


    function jac_x = JF_x(obj) 

        theta  = x(3);
        jac_x    = [ cos(theta), -sin(theta), 0; ...
                   sin(theta),  cos(theta), 0; ...
                   0,           0,          1  ];
    end

    function jac_n = JF_n(obj) 

        theta  = x(3);
        jac_n    = [ cos(theta), -sin(theta), 0; ...
                   sin(theta),  cos(theta), 0; ...
                   0,           0,          1  ];
    end
   
        

    
    function next_state = update_step(obj, odometry)

        next_state = obj.x + JF_x()*odometry.dX;
        obj.x = next_state;
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