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
   

%  ____       _            _         __  __                _                   
% |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
% | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
% |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
% |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
%
% Here are defined auxiliary functions used in the public members or for other simpler computations



end % methods
end % Laserscan class