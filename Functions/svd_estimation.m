
% Function that given two laserscans "ls1" and "ls2" returns the attitude difference based on the
% singular value decomposition
function [alpha] = svd_estimation(ls1, ls2)

    X1 = ls1.cartesian - mean(ls1.cartesian, 1);    % remove mean value from each row
    X2 = ls2.cartesian - mean(ls2.cartesian, 1);

    [U1, S1, V1] = svd(X1);
    [U2, S2, V2] = svd(X2);

    R12 = V2 * V1';
    R21 = V1 * V2';

    alpha_12 = atan2(R12(2, 1), R12(1, 1));
    alpha_21 = atan2(R21(2, 1), R21(1, 1));

    % For debug purposes
    % fprintf("alpha_12 = %f\n", alpha_12);
    % fprintf("alpha_21 = %f\n", alpha_21);
    % disp(V1)
    % disp(V2)
    
    alpha = - (alpha_12 - alpha_21) / 2;
end