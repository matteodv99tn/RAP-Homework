
% Feature extraction based on seeded region growing based on the article
% https://journals.sagepub.com/doi/pdf/10.1177/1729881418755245
%
% Inputs are:
%   - laserscan:    a object containing all laserscan data (in particular cartesian coordinates) and
%                   angles of the measurement, see load_data.m);
%   - conf:         configuration struct of the algorithm containg the parameters

function extracted_feature = extract_feature_v2(laserscan, conf)
    
    % First we want to generate the seeds. We increment the vector "overall_seeds" by adding the 
    % pair of starting and ending value of the seed
    overall_seeds = [];

    i = 2;
    while i < (conf.Np - conf.Pmin)

        % algorithm 1: seed detection
        j = i + conf.Snum;
        disp(['Comparing points ', num2str(i), '-', num2str(j)])
        [is_seed, seed_ij] = seed_segment(laserscan, i, j, conf);

        if is_seed % -> algorithm 2: seed growing

            disp('-> Is seed')

            [succeded, seed_new, i_new, j_new] = region_growing(laserscan, seed_ij, i, j, conf);
            
            if succeded
                disp('ADDING A SEED')
                overall_seeds(end+1, :)  = [i_new; j_new];
            end

            i = j_new;
        end
        
        i = i+1;
    end

    extracted_feature = overall_seeds;
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
%   - i, j:         starting/ending index for the seeding
%   - conf:         configuration parameters structure
% 
% Outputs
%   - flag:         boolean for succeded/non succeeded seed segmentation
%   - seed_ij:      [a, b, c] paremeters of the seeded line; -1 if was not a correct seed
function [flag, seed_ij] = seed_segment(laserscan, i, j, conf)

    flag        = true;     % by default we assume that the two points are corresponding to a
                            % feasible line
    seed_ij     = -1;       % set by default all values to -1 (in case of not a proper line)

    line    = seed(laserscan, i, j); % least square line fitting
    
    for k = i:j % for each point in the seed

        Ppred = predicted_point(line, laserscan.Theta(k));
        
        if point_point_distance(laserscan.scan(k,:), Ppred) > conf.delta                                           
            flag = false;
            break;
        end

        if line_point_distance(line, Ppred) > conf.eps 
            flag = false;
            break;
        end  
    end
    
    if flag
        seed_ij = line;
    end    
end


%% Algorithm 2: Region Growing
% function that returns the parameters of a line: coefficients, starting and ending points. 
function [grow_succedeed, coeffs_line, Pb, Pf] = region_growing(laserscan, seed_ij, i, j, conf)
    % Initialization
    coeffs_line = seed_ij;
    Ll = 0;
    Pl = 0;
    Pf = j + 1;
    Pb = i - 1;

    while line_point_distance(coeffs_line, laserscan.scan(Pf,:)) < conf.eps
        if Pf > Np
            break;
        else
            coeffs_line = seed(laserscan, Pb, Pf);
        end
        Pf = Pf + 1;
    end
    Pf = Pf - 1;

    while line_point_distance(coeffs_line, laserscan.scan(Pb,:)) < conf.eps
        if Pb < 1
            break;
        else
            coeffs_line = seed(laserscan, Pb, Pf);
        end
        Pb = Pb - 1;
    end
    Pb = Pb + 1;
    
    Ll = point_point_distance(laserscan.scan(Pb,:),laserscan.scan(Pf,:));
    Pl = Pf - Pb;
    if Ll >= conf.Lmin && Pl >= conf.Pmin
        coeffs_line     = seed(laserscan, Pb, Pf);
        grow_succedeed  = true;
        return;
    else
        grow_succedeed  = false;
        return;
    end
end


%% Algoritm 3: overlap region processing
% function [line_i, m1, n1, line_j, m2, n2] = overlap_region(laserscan, seed_i, seed_j)

    
        
%     Pb1 = seed_i(1);  % starting index of line i 
%     Pf1 = seed_i(2);  % ending index of line i  
%     Pb2 = seed_j(1);  % starting index of line j
%     Pf2 = seed_j(2);  % ending index of line j  

%     if m2 <= n1
%         for k = m2:n1
%             Pk      = laserscan.scan(k,:);
%             line_i  = seed(laserscan.scan(m1,:), laserscan.scan(n1,:));
%             line_j  = seed(laserscan.scan(m2,:), laserscan.scan(n2,:));
%             dk_i    = line_point_distance(line_i, Pk);
%             dk_j    = line_point_distance(line_j, Pk);
%             if dk_j < dk_i
%                 break;
%             end
%         end
%         n1 = k - 1;
%         m2 = k;
%     else
%         break;
%     end
%     line_i  = seed(laserscan.scan(m1,:), laserscan.scan(n1,:));
%     line_j  = seed(laserscan.scan(m2,:), laserscan.scan(n2,:));
    
% end


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
    
    A = [laserscan.scan(i:j, :), ones(j-i+1, 1)];
    b = zeros(j-i+1, 1);
    
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

