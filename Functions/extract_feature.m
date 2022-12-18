
% Feature extraction based on seeded region growing based on the article
% https://journals.sagepub.com/doi/pdf/10.1177/1729881418755245
%
% Inputs are:
%   - laserscan:    a object containing all laserscan data (in particular cartesian coordinates) and
%                   angles of the measurement, see load_data.m);
%   - conf:         configuration struct of the algorithm containg the parameters

function extracted_feature = extract_feature(laserscan, conf)

    % Copy configuration files
    Np      = conf.Np;  
    eps     = conf.eps;  
    delta   = conf.delta;
    Snum    = conf.Snum; 
    Pmin    = conf.Pmin; 
    Lmin    = conf.Lmin; 

    %% Alg. 1: seed-segment detection
    % Firstly we want to generate the seeds. We increment the vector "overall_seeds" by adding the 
    % pair of starting and ending value of the seed
    overall_seeds = [];

    for i = 1:Np-Pmin

        j       = i + Snum;
        line    = seed(laserscan.scan[i], laserscan.scan[j]);
        is_seed = true;

        for k = i:j

            Ppred   = predicted_point(line, laserscan.Theta[k]);
            d       = point_point_distance(laserscan.scan[k], Ppred);            
            if d > delta 
                is_seed = false;
                break;
            end

            d       = line_point_distance(line, Ppred);
            if d > eps 
                is_seed = false;
                break;
            end            
        end

        if is_seed
            overall_seeds(end+1) = [i; j];
        end

        % maybe it should be added ?
        % i = j;

    end


end


% Given two points p1, p2 it computes the 3 coefficients of the line that joint them in the form
%       a*x + b*y + c = 0
% as reported in the paper; source for this formula:
% https://math.stackexchange.com/questions/637922/how-can-i-find-coefficients-a-b-c-given-two-points
function coeffs = seed(p1, p2)

    x1  = p1[1];
    y1  = p1[1];
    x2  = p2[1];
    y2  = p2[1];

    a   = y1 - y2;
    b   = x2 - x1;
    c   = x1*y2 - x2*y1;

    coeffs = [a, b, c];

end


% compute predicted point; based on the paper formula
function Ppred = predicted_point(line_coeff, theta)

    a   = line_coeff[1];
    b   = line_coeff[1];
    c   = line_coeff[1];

    den = a*cos(theta) + b*sin(theta);

    Ppred = [
            - c * cos(theta) / den;
            - c * sin(theta) / den;
        ];

end


% computes the distance between two points
function d = point_point_distance(p1, p2)

    x1  = p1[1];
    y1  = p1[2];
    x2  = p2[1];
    y2  = p2[2];

    d   = sqrt( (x1-x2)^2 + (y1-y2)^2 );

end


% computes the distance from a line and a point
function d = line_point_distance(line_coeff, point)

    a   = line_coeff[1];
    b   = line_coeff[1];
    c   = line_coeff[1];
    x   = point[1];
    y   = point[2];

    num = abs(a*x + b*y +c);
    den = sqrt(a^2 + b^2);

    d   = num / den;

end

