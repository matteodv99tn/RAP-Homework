
% Feature extraction based on seeded region growing based on the article
% https://journals.sagepub.com/doi/pdf/10.1177/1729881418755245
%
% Inputs are:
%   - laserscan:    a object containing all laserscan data (in particular cartesian coordinates) and
%                   angles of the measurement, see load_data.m);
%   - conf:         configuration struct of the algorithm containg the parameters

function extracted_feature = extract_feature(laserscan,conf)

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

    % Ideal seed-segment should satisfy following two requirements:
    %   1) Distance point -> point: distance from every point in the seed-segment to its predicted position also should be less than a given threshold
    %   2) Distance point -> line : The distance from every point in the seed-segment to the fitting straight line should be less than a given threshold
    %
    
    overall_seeds = [];
    row = 1;

    for i = 1:Np-Pmin

        j       = i + Snum;
        line    = seed([laserscan.scan(i,1),laserscan.scan(i,2)], [laserscan.scan(j,1),laserscan.scan(j,2)]); % fitting line
        is_seed = true;

        for k = i:j

            Ppred   = predicted_point(line, laserscan.Theta(k));
            d1      = point_point_distance([laserscan.scan(k,1),laserscan.scan(k,2)], Ppred);            
            % 1) requirement
            if d1 > delta                                           
                is_seed = false;
                break;
            end

            d2      = line_point_distance(line, Ppred);
            % 2) requirement
            if d2 > eps 
                is_seed = false;
                break;
            end            
        end

        if is_seed == true
            %% Alghoritm 2: Region growing --> it's excited here, when seed-segment detection is successful
            [coeffs_line, Pb, Pf] = region_growing(line, i, j, Np, Pmin, Lmin, eps);
            overall_seeds(row,1) = Pb;
            overall_seeds(row,2) = Pf;
            row = row + 1;
        end

        % maybe it should be added ?
        % i = j;
    end

    %% Alghoritm 3: Overlap region processing
    % compute new line-segments without overlap region
    extracted_feature = overlap_region(row,overall_seeds); 
end


%% Implementation of the 2nd alghoritm: region growing
% function that returns the parameters of a line: coefficients, starting and ending points. 
function [coeffs_line, Pb, Pf] = region_growing(seed_ij, i, j, Np, Pmin, Lmin, eps)
    % Initialization
    coeffs_line = seed_ij;
    Ll = 0;
    Pl = 0;
    Pf = j + i;
    Pb = i - 1;

  
    % Implementation
    dist = line_point_distance(coeffs_line,[laserscan.scan(Pf,1),laserscan.scan(Pf,2)]);
    while dist < eps
        if Pf > Np
            break;
        else
            coeffs_line = seed([laserscan.scan(Pb,1),laserscan.scan(Pb,2)],[laserscan.scan(Pf,1),laserscan.scan(Pf,2)]);
        end
        Pf = Pf + 1;
    end
    Pf = Pf - 1;

    dist = line_point_distance(coeffs_line,[laserscan.scan(Pb,1),laserscan.scan(Pb,2)]);
    while dist < eps
        if Pb < 1
            break;
        else
            coeffs_line = seed([laserscan.scan(Pb,1),laserscan.scan(Pb,2)],[laserscan.scan(Pf,1),laserscan.scan(Pf,2)]);
        end
        Pb = Pb - 1;
    end
    Pb = Pb + 1;
    
    Ll = point_point_distance([laserscan.scan(Pb,1),laserscan.scan(Pb,2)],[laserscan.scan(Pf,1),laserscan.scan(Pf,2)]);
    Pl = Pf - Pb;
    if Ll >= Lmin && Pl >= Pmin
        coeffs_line = seed([laserscan.scan(Pb,1),laserscan.scan(Pb,2)],[laserscan.scan(Pf,1),laserscan.scan(Pf,2)]);
    %else ? --> if it is not matching the requirements what we have to
    %return?
    end
end


%% Implementation of the 3rd alghoritm: overlap region processing
% function returns ine segments without overlap region
function new_lines = overlap_region(row,segments)
    new_lines = [];

    for i = 1:row-1
        j = i + 1;
        
        m1 = segments(i,1);  % StartPoint Index of Line i 
        n1 = segments(i,2);  % EndPoint Index of Line i 
        m2 = segments(j,1);  % StartPoint Index of Line j
        n2 = segments(j,2);  % EndPoint Index of Line j

        if m2 <= n1
            for k = m2:n1
                Pk = [laserscan.scan(k,1),laserscan.scan(k,2)];
                line_i = seed([laserscan.scan(m1,1),laserscan.scan(m1,2)],[laserscan.scan(n1,1),laserscan.scan(n1,2)]);
                line_j = seed([laserscan.scan(m1,1),laserscan.scan(m1,2)],[laserscan.scan(n1,1),laserscan.scan(n1,2)]);
                dk_i = line_point_distance(line_i, Pk);
                dk_j = line_point_distance(line_j, Pk);
                if dk_j < dk_i
                    break;
                end
            end
            n1 = k - 1;
            m2 = k;
        end
        new_lines(i,1) = m1;
        new_lines(i,2) = n1;
        if i == (row-1)
            new_lines(row,1) = m2;
            new_lines(row,2) = n2;
        end
    end
    
end


% Given two points p1, p2 it computes the 3 coefficients of the line that joint them in the form
%       a*x + b*y + c = 0
% as reported in the paper; source for this formula:
% https://math.stackexchange.com/questions/637922/how-can-i-find-coefficients-a-b-c-given-two-points
function [coeffs, pb, pf] = seed(p1, p2)

    x1  = p1(1);
    y1  = p1(2);
    x2  = p2(1);
    y2  = p2(2);

    a   = y1 - y2;
    b   = x2 - x1;
    c   = x1*y2 - x2*y1;

    coeffs = [a, b, c];
    pb = p1;
    pf = p2;

end


% compute predicted point; based on the paper formula
function Ppred = predicted_point(line_coeff, theta)

    a   = line_coeff(1);
    b   = line_coeff(2);
    c   = line_coeff(3);

    den = a*cos(theta) + b*sin(theta);

    Ppred = [
            - c * cos(theta) / den;
            - c * sin(theta) / den;
        ];

end


% computes the distance between two points
function d = point_point_distance(p1, p2)

    x1  = p1(1);
    y1  = p1(2);
    x2  = p2(1);
    y2  = p2(2);

    d   = sqrt( (x1-x2)^2 + (y1-y2)^2 );

end


% computes the distance from a line and a point
function d = line_point_distance(line_coeff, point)

    a   = line_coeff(1);
    b   = line_coeff(2);
    c   = line_coeff(3);
    x   = point(1);
    y   = point(2);

    num = abs(a*x + b*y +c);
    den = sqrt(a^2 + b^2);

    d   = num / den;

end

