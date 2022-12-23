
% Feature extraction based on seeded region growing based on the article
% https://journals.sagepub.com/doi/pdf/10.1177/1729881418755245
%
% Inputs are:
%   - laserscan:    a object containing all laserscan data (in particular cartesian coordinates) and
%                   angles of the measurement, see load_data.m);
%   - conf:         configuration struct of the algorithm containg the parameters

function extracted_feature = extract_feature(laserscan, conf)
    
    % First we want to generate the seeds. We increment the vector "overall_seeds" by adding the 
    % pair of starting and ending value of the seed
    overall_seeds = [];
    
    % Copy configuration files
    Np      = conf.Np;  
    eps     = conf.eps;  
    delta   = conf.delta;
    Snum    = conf.Snum; 
    Pmin    = conf.Pmin; 
    Lmin    = conf.Lmin; 

    %% Alghoritm 1: seed-segment detection
    % Ideal seed-segment should satisfy following two requirements:
    %   1) Distance point -> point: distance from every point in the seed-segment to its predicted position also should be less than a given threshold
    %   2) Distance point -> line : The distance from every point in the seed-segment to the fitting straight line should be less than a given threshold
  
    [is_seed, seed_ij, index_i, index_j] = seed_segment(laserscan, conf);
    

    % Now I check if seed-segment detection was successful and in this case
    % I can continue with region growing, otherwise I exit from the
    % function
    if is_seed
        %% Alghoritm 2: Region growing 
        [succeeded, coeffs_line, Pb, Pf] = region_growing(laserscan, seed_ij, index_i, index_j, Np, Pmin, Lmin, eps);
            
        overall_seeds(end+1,:) = [Pb; Pf];
    else
        disp('In this laserscan I did not succeed in the Alghoritm 1')
        return      
    end

    %% Alghoritm 3: Overlap region processing
    % compute new line-segments without overlap region
    
    % WHAT DO WE HAVE TO RETURN FROM ALGHORITM 3?? 2 SEGMENTS OR ONE?????
    
    [line_i, m1, n1, line_j, m2, n2] = overlap_region(overall_seeds, laserscan);
    
    extracted_feature(end+1,:) = [];
end

%     _    _                  _ _   _                   
%    / \  | | __ _  ___  _ __(_) |_| |__  _ __ ___  ___ 
%   / _ \ | |/ _` |/ _ \| '__| | __| '_ \| '_ ` _ \/ __|
%  / ___ \| | (_| | (_) | |  | | |_| | | | | | | | \__ \
% /_/   \_\_|\__, |\___/|_|  |_|\__|_| |_|_| |_| |_|___/
%            |___/                                      
%
% Implementation of the 3 algorithms for the feature extraction


%% Implementation of the 1st alghoritm: Seed-segment detection

%% Algorithm 1: Seed-Segment Detection
% 
% Inputs
%   - laserscan:    data structure containing all information of the laserscan to extract feature to
%   - conf:         configuration parameters structure
% 
% Outputs
%   - flag:         boolean for succeded/non succeeded seed segmentation
%   - seed_ij:      [a, b, c] paremeters of the seeded line; -1 if was not a correct seed
%   - index_i:      starting index of the seed; -1 if was not a correct seed
%   - index_j:      ending index of the seed; -1 if was not a correct seed
function [flag, seed_ij, index_i, index_j] = seed_segment(laserscan, conf)

    flag        = true;     % by default we assume that the two points are corresponding to a
                            % feasible line
    seed_ij     = -1;       % set by default all values to -1 (in case of not a proper line)
    index_i     = -1;
    index_j     = -1;

    for i = 1:(conf.Np-conf.Pmin)

        j       = i + conf.Snum;
        line    = seed(laserscan, i, j); % least square line fitting
        
        for k = i:j

            Ppred   = predicted_point(line, laserscan.Theta(k));
            
            % 1) requirement
            if point_point_distance(laserscan.scan(k,:), Ppred); > delta                                           
                flag = false;
                break;
            end

            % 2) requirement
            if line_point_distance(line, Ppred) > eps 
                flag = false;
                break;
            end  
        end
        
        if flag
            seed_ij = line;
            index_i = i;
            index_j = j;
            return;
        end    
    end
end


%% Algorithm 2: Region Growing
% function that returns the parameters of a line: coefficients, starting and ending points. 
function [grow_succedeed, coeffs_line, Pb, Pf] = region_growing(laserscan, seed_ij, i, j, Np, Pmin, Lmin, eps)
    % Initialization
    coeffs_line = seed_ij;
    Ll = 0;
    Pl = 0;
    Pf = j + i;
    Pb = i - 1;

    while line_point_distance(coeffs_line,laserscan.scan(Pf,:)) < eps
        if Pf > Np
            break;
        else
            coeffs_line = seed(laserscan.scan(Pb,:),laserscan.scan(Pf,:));
        end
        Pf = Pf + 1;
    end
    Pf = Pf - 1;

    while line_point_distance(coeffs_line,laserscan.scan(Pb,:)) < eps
        if Pb < 1
            break;
        else
            coeffs_line = seed(laserscan.scan(Pb,:),laserscan.scan(Pf,:));
        end
        Pb = Pb - 1;
    end
    Pb = Pb + 1;
    
    Ll = point_point_distance(laserscan.scan(Pb,:),laserscan.scan(Pf,:));
    Pl = Pf - Pb;
    if Ll >= Lmin && Pl >= Pmin
        coeffs_line = seed(laserscan.scan(Pb,:),laserscan.scan(Pf,:));
        grow_succedeed = true;
        return;
    else
        grow_succedeed = false;
        return;
    end
end


%% Implementation of the 3rd alghoritm: overlap region processing
% function returns line segments without overlap region
function [line_i, m1, n1, line_j, m2, n2] = overlap_region(overall_seeds, laserscan)
    num_seeds = size(overall_seeds,1);

    for i = 1:num_seeds-1
        j   = i + 1;
        
        m1  = overall_seeds(i,1);  % StartPoint Index of Line i 
        n1  = overall_seeds(i,2);  % EndPoint Index of Line i 
        m2  = overall_seeds(j,1);  % StartPoint Index of Line j
        n2  = overall_seeds(j,2);  % EndPoint Index of Line j

        if m2 <= n1
            for k = m2:n1
                Pk      = laserscan.scan(k,:);
                line_i  = seed(laserscan.scan(m1,:), laserscan.scan(n1,:));
                line_j  = seed(laserscan.scan(m2,:), laserscan.scan(n2,:));
                dk_i    = line_point_distance(line_i, Pk);
                dk_j    = line_point_distance(line_j, Pk);
                if dk_j < dk_i
                    break;
                end
            end
            n1 = k - 1;
            m2 = k;
        else
            break;
        end
        line_i  = seed(laserscan.scan(m1,:), laserscan.scan(n1,:));
        line_j  = seed(laserscan.scan(m2,:), laserscan.scan(n2,:));
    end
    
end


%  _____                 _   _                 
% |  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
% | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
% |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
% |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%                                              
% Implementation of functions used in the main algorithm


% Given two points p1, p2 it computes the 3 coefficients of the line that joint them in the form
%       a*x + b*y + c = 0
% as reported in the paper; source for this formula:
% https://math.stackexchange.com/questions/637922/how-can-i-find-coefficients-a-b-c-given-two-points
% function coeffs = seed(p1, p2)

%     x1  = p1(1);
%     y1  = p1(2);
%     x2  = p2(1);
%     y2  = p2(2);

%     a   = y1 - y2;
%     b   = x2 - x1;
%     c   = x1*y2 - x2*y1;

%     coeffs = [a, b, c];
% end

function coeffs = seed(laserscan, i, j) % least square seeding

    A = [   ones(j-i, 1),   laserscan.xscan(i:j)'  ];
    b = laserscan.yscan(i:j)';

    coeffs = pinv(A) * b;

end


% compute predicted point; based on the paper formula
function Ppred = predicted_point(line_coeff, theta)

    a   = line_coeff(1);
    b   = line_coeff(2);
    c   = line_coeff(3);
    den = a*cos(theta) + b*sin(theta);

    Ppred = [ - c * cos(theta) / den  ;...
              - c * sin(theta) / den  ];

end


% computes the distance between two points
function d = point_point_distance(p1, p2)

    d = sqrt(sumsqr(p1 - p2));

end


% computes the distance from a line and a point as reported in the paper
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

