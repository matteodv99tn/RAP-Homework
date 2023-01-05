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

    % see eq. (18) in 
    % https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf
    % Projecting the absolute coordinate of a landmark into the reference frame of the robot.
    % The function return also the jacobians w.r.t. the state of the robot and the state of the landmark
    function [h, Jh_x_robot, Jh_x_landmark] = landmark_observation(obj, landmark)
        
        % landmark state
        x_land = landmark.x(1);
        y_land = landmark.x(2);

        % robot state
        x_rob = obj.x(1);
        y_rob = obj.x(2);
        t_rob = obj.x(3);

        % Transformation of reference frame from global (x_land, y_land) to local (xp, yp)
        % x_land = x_rob + xp * cos(t_rob) - yp * sin(t_rob) 
        % y_land = y_rob + xp * sin(t_rob) - yp * cos(t_rob)
        %
        % local coordinates are:
        xp = cos(t_rob) * (x_land - x_rob) + sin(t_rob) * (y_land - y_rob);
        yp = cos(t_rob) * (y_land - y_rob) + sin(t_rob) * (- x_land + x_rob);
        
        h = [xp;yp];

        % Jacobian w.r.t the robot state
        j11 = - cos(t_rob);
        j12 = - sin(t_rob);
        j13 = sin(t_rob) * (x_land - x_rob) - cos(t_rob) * (y_land - y_rob);
        j21 = sin(t_rob);
        j22 = - cos(t_rob);
        j23 = - sin(t_rob) * (y_land - y_rob) + cos(t_rob) * (- x_land + x_rob);

        Jh_x_robot = [j11,j12,j13;
                      j21,j22,j23];

        % Jacobian w.r.t the landmark state        
        j11 = cos(t_rob);
        j12 = sin(t_rob);
        j21 = - sin(t_rob);
        j22 = cos(t_rob);

        Jh_x_landmark = [j11,j12;
                         j21,j22];

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